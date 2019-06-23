/**
  ******************************************************************************
  * @file    usart_xbee_test_terminal/main.c
  * @author  Takaaki Kitamori
  * @version V5.2.0
  * @date    12/04/2012
  * @brief   Main program body
  ******************************************************************************
  * @copy
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "scanf.h"
#include "delay.h"
#include <string.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TX_POINTER_BUFFER       255
#define STR_BUFFER_SIZE         255
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int8_t *TxData[TX_POINTER_BUFFER];
uint8_t subscript_in_progress = 0, subscript_to_add = 0, Rxindex=0, RxData;
uint8_t Time[STR_BUFFER_SIZE];
int32_t RTC0=0;
uint8_t ComBuffer[0];

/* Private function prototypes -----------------------------------------------*/
void NVIC_XBee_Configuration(void);
void GPIO_Configuration(void);
void GPIO_XBee_Configuration(void);
void XBee_Port_Configuration(uint32_t baudrate);
void FullRemap_XBee_Port_Configuration(void);
void XBee_Port_Send_String(int8_t String[]);
void USART3_IRQHandler(void);//入れてみたけど効果なし
void XBee_Port_DMA_Configuration(uint32_t Memory_Address, uint16_t Buffer_Size);
void Convert_Number(uint8_t String[], int32_t Number);
int32_t  Convert_String(uint8_t String[], int32_t Number);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_XBee_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable DMA1 channel7 IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the GPIO Pins.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
/* Supply APB2 clock */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //リセットやクロック等の初期化
GPIO_InitTypeDef GPIO_InitStructure;

/* Configure GPIOX_0: output push-pull */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //使うピンの宣言
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //出力ピンとして用いる
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //ピンの制御スピード？
GPIO_Init(GPIOB, &GPIO_InitStructure);  //GPIOの初期化？
}
/**
  * @brief  Configure the GPIO Pins.
  * @param  None
  * @retval : None
  */
void GPIO_XBee_Configuration(void)
{
  /* Supply APB2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure GPIOY_0: output push-pull */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  Configure the USART3.
  * @param  None
  * @retval : None
  */
void XBee_Port_Configuration(uint32_t baudrate)
{

  /* Supply APB1 clock */
  RCC_APB1PeriphClockCmd(USART3_RCC , ENABLE);
  RCC_APB2PeriphClockCmd(USART3_GPIO_RCC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  /* Configure USART3 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART3_PORT, &GPIO_InitStructure);
  /* Configure USART3 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART3_PORT, &GPIO_InitStructure);

#if defined (PARTIAL_REMAP_USART3)
PartialRemap_USART3_Configuration();
#elif defined (FULL_REMAP_USART3)
FullRemap_USART3_Configuration();
#endif

  /* Disable the USART3 */
  USART_Cmd(USART3, DISABLE);

  /* USART3 configuration ------------------------------------------------------*/
    /* USART3 configured as follow:
        - BaudRate = set by argument
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
  USART_InitTypeDef USART_InitStructure;
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStructure);

  /* Enable USART3 DMA TX request */
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

  //Enable USART3 RX empty interrupt
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  /* Enable the USART3 */
  USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval None
  */
void DMA_Configuration(uint32_t Memory_Address, uint16_t Buffer_Size)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* DMA channel for USART3 (DMA1_Channel2) Config */
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Memory_Address;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = Buffer_Size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  /* Enable DMA1 Channel2 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
}

/**
  * @brief  Send Strings via USART2
  * @param  String: Array containing string to be sent
  * @retval : None
  */
void XBee_Port_Send_String(int8_t String[])
{
  uint8_t idle_flag = 0;

  // check the needs to enable and start DMA transfer
  if (subscript_in_progress == subscript_to_add) idle_flag = 1;

  //if subscript comes around and get to one in progress, then wait.
  if (subscript_in_progress == subscript_to_add + 1 || ( subscript_to_add == TX_POINTER_BUFFER - 1 && subscript_in_progress == 0) )
    {
      while(subscript_in_progress == subscript_to_add + 1 || ( subscript_to_add == TX_POINTER_BUFFER - 1 && subscript_in_progress == 0)){}
    }

  // set string pointer to buffer and increment subscript
  TxData[subscript_to_add++] = (int8_t *)String;

  //if subscript reaches end make to go back to front
  if (subscript_to_add == TX_POINTER_BUFFER)
    {
      subscript_to_add = 0;
    }

  // enable and start DMA transfer
  if (idle_flag != 0)
    {
      DMA_Configuration((uint32_t)TxData[subscript_in_progress], strlen(TxData[subscript_in_progress]));
      /* Enable USARTy DMA TX Channel */
      DMA_Cmd(DMA1_Channel2, ENABLE);
    }
}


/**
  * @brief  This function handles DMA1 Channel 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* Test on DMA1 Channel2 Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_IT_TC2))
  {
    /* Disable USARTy DMA TX Channel */
    DMA_Cmd(DMA1_Channel2, DISABLE);

    // move to next string
    subscript_in_progress++;
    // if pointer reached the end of pointer buffer, make it go to front
    if (subscript_in_progress == TX_POINTER_BUFFER)
      {
        subscript_in_progress = 0;
      }

    // if pointer processing reaches to pointer the next message to be add
    // stops generate interrupt and stop sending.
    if (subscript_in_progress != subscript_to_add)
      {
        DMA_Configuration((uint32_t)TxData[subscript_in_progress], strlen(TxData[subscript_in_progress]));
        /* Enable USARTy DMA TX Channel */
        DMA_Cmd(DMA1_Channel2, ENABLE);
      }
    /* Clear DMA1 Channel2 Transfer Complete interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TC2);
  }
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)//文字列初期化する必要あり?
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
      RxData=(uint8_t)USART_ReceiveData(USART3);
      ComBuffer[0]=RxData;
      //cprintf(ComBuffer);
      if (RxData == 119) forward();
      else if (RxData == 97) turnleft();
      else if (RxData == 100) turnright();
      else if (RxData == 115) back();
      else if (RxData == 101) gun();
      //エコーバックの有無
      //XBee_Port_Send_String(ComBuffer);
      USART_ClearFlag(USART3, USART_IT_RXNE);
    }
}

void Convert_Number(uint8_t String[], int32_t Number){
  uint8_t index=0;
  uint8_t j=0;
  uint8_t num_str[15];

  if(Number<0){
      String[index++]='-';
      Number*=-1;
  }
  else{
      String[index++]='+';
  }
  num_str[j++]=('0'+(uint8_t)(Number%10));
  Number/=10;
  while(Number>0){
      num_str[j++]=('0'+(uint8_t)(Number%10));
      Number/=10;
  }
  while(j>0)  String[index++]=num_str[--j];

  String[index++]='^';
}

int32_t Convert_String(uint8_t String[], int32_t Number)
{
  uint8_t index=1;
//  uint8_t j=0, a=0, b=0;
//  uint8_t num_str[15];
//  uint32_t temp=0;

  Number=0;
  while(String[index]!='^'){
      Number=Number*10+String[index]-48;
      index++;
  }

  if(String[0]=='-'){
      Number*=-1;
  }
  else{
      Number*=1;
  }

  return Number;
}
