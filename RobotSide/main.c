/**
  ******************************************************************************
  * @file    kikaisouzou group8 program
  * @author  onanism mantaro
  * @version V1.0.0
  * @date    2012/12/20
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *�@�̑�
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "com_config.h"
#include "delay.h"
#include "XBee.h"

uint8_t Buffer[0];
uint8_t ComRxData;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CHATTER  2
#define SAMPLE_TIMES 3
#define SAMPLE_INTERVAL 10


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const int8_t Welcome_Message[] = "\r\nHellow Cortex-M3/STM32 World!\r\n"
                "Expand your creativity and enjoy making.\r\n\r\n"
                "I report the distance between PSD sensor and reflective object.\r\n"
                "10 - 30 cm is Red / 30 - 50 cm is Yellow / 50 - cm is Green.\r\n";
__IO uint16_t ARR_Val = 100;
//***PSD�Z���T�p�֐�
int16_t ADC_Value1;
int16_t ADC_Value2;
int16_t ADC_Value3;
int16_t ADC_Value4;
uint8_t count1[3];
uint8_t count2[3];
uint8_t count3[3];
uint8_t count4[3];
uint8_t State1 = 4;
uint8_t State2 = 4;
uint8_t State3 = 4;
uint8_t State4 = 4;
__IO int16_t ADCConvertedValue[4];


//***���~�b�^�X�C�b�`�p�֐�
int8_t SW0_previous_state = 0, SW0_present_state = 0,
       SW0_scan_count = 0, SW0_flag = RESET;
int8_t SW1_previous_state = 0, SW1_present_state = 0,
       SW1_scan_count = 0, SW1_flag = RESET;
//********����**********//
//***PSD���������ݒ�(�傫������Ɣ������߂��Ȃ�)(1100���f�t�H)
int16_t Red_zone = 1100;
//***�E����PSD�������̉�]�p����
int16_t rightangle=900;
//***������PSD�������̉�]�p����
int16_t leftangle=400;
//***������PSD�������̉�]�p����
int16_t rotateangle=1500;
//***�I��T���Ƃ��̊p�x����
int16_t searchangle=300;
//***�O�i����
int16_t forwardtime=500;
//***��ގ���
int16_t backtime=500;
//***�e���[�^��]����(1000���f�t�H)
int16_t guntime=850;
//***�T���Ƃ��̉�]���x(50~100)
int16_t searchspeed=100;

/* Private function prototypes -----------------------------------------------*/
//�e�평���ݒ�
void RCC_Configuration(void);
void NVIC_Configuration(void);
void ADC_Configuration(void);
//PSD�pTIM
void TIM1_Configuration(void);
//�X�C�b�`�pTIM
void TIM4_Configuration(void);
void ADC1_2_IRQHandler(void);
void SW0_is_released(void);
void SW0_is_pressed(void);
void SW1_is_released(void);
void SW1_is_pressed(void);
//PWM�o�͗pTIM
void TIM3_Configuration(void);
/**�e�퓮�삽��**/
void forward(void);
void back(void);
void turnright(void);
void turnleft(void);
void rotate(void);
void gun(void);
void bend(void);
void stretch(void);
void stop(void);
void turnsearch(void);
// *
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{

  // Configure board specific setting
  BoardInit();
  // Setting up COM port for Print function
  //COM_Configuration();
  /* System Clocks re-configuration **********************************************/
  RCC_Configuration();

  //Configure ADC
  ADC_Configuration();
  //Configure TIM1
  TIM1_Configuration();
  //Configure TIM3
  TIM3_Configuration();
  //Configure TIM4
  TIM4_Configuration();

  TIM_SetCompare1(TIM3,50);
  TIM_SetCompare2(TIM3,50);
  TIM_SetCompare3(TIM3,50);
  TIM_SetCompare4(TIM3,50);

  // Wait XBee module to wake up
  delay_ms(1500);
  //NVICXBEE
  NVIC_XBee_Configuration();

  // Do reset XBee
  GPIO_XBee_Configuration();
  GPIO_ResetBits(GPIOB,GPIO_Pin_8);
  delay_ms(100);
  GPIO_SetBits(GPIOB,GPIO_Pin_8);

  // set USART3 speed to XBee default baudrate(9600bps)
  XBee_Port_Configuration(9600);

  // Wait XBee module to wake up
  delay_ms(1000);

  XBee_Port_Send_String("Ready");
  XBee_Port_Send_String("@");

  XBee_Port_Send_String("GO");
  XBee_Port_Send_String("@");

  //Configure NVIC
  NVIC_Configuration();

  //Send welcome messages
  //cprintf(Welcome_Message);
  while (1){
  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);
}

/**
  * @brief  Configure ADC1
  * @param  None
  * @retval : None
  */
void ADC_Configuration(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  //�N���b�N����
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(
      /*�X�C�b�`�p����*/GPIOY_4_RCC |GPIOY_5_RCC |
      /*LED�p����*/GPIOX_RCC|
      /*PSD�p����*/RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |RCC_APB2Periph_ADC3 |ADC_IN0_3_GPIO_RCC |ADC_IN4_7_GPIO_RCC |ADC_IN10_15_GPIO_RCC | ADC_IN8_9_GPIO_RCC |GPIOY_0_RCC
      , ENABLE);

  //�X�C�b�`�p(PA4)
  GPIO_InitStructure.GPIO_Pin = GPIOY_4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOY_4_PORT, &GPIO_InitStructure);

  //�X�C�b�`�p(PA5)
  GPIO_InitStructure.GPIO_Pin = GPIOY_5_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOY_5_PORT, &GPIO_InitStructure);

  //PSD�p(PA0)
  GPIO_InitStructure.GPIO_Pin = ADC123_IN0_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(ADC123_IN0_PORT, &GPIO_InitStructure);

  //PSD�p(PA1)
  GPIO_InitStructure.GPIO_Pin = ADC123_IN1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(ADC123_IN1_PORT, &GPIO_InitStructure);

  //PSD�p(PA2)
  GPIO_InitStructure.GPIO_Pin = ADC123_IN2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(ADC123_IN2_PORT, &GPIO_InitStructure);

  //PSD�p(PA3)
  GPIO_InitStructure.GPIO_Pin = ADC123_IN3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(ADC123_IN3_PORT, &GPIO_InitStructure);

  /* GPIO Configuration: output push-pull */
  //LED�p(PB3~6)
  GPIO_InitStructure.GPIO_Pin = GPIOX_4_PIN | GPIOX_2_PIN | GPIOX_3_PIN|GPIOX_5_PIN | GPIOX_6_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOX_PORT, &GPIO_InitStructure);

  /* DMA1 channel1 configuration(PSD�p) ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable DMA1 Channel1 Transfer Complete interrupt */
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

  /*ADC12_IN8_PIN~PB0,*/

  ADC_DeInit(ADC1);

  /* ADC1 Configuration (PSD�p)------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC123_IN0_CH, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC123_IN1_CH, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC123_IN2_CH, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC123_IN3_CH, 4, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE);

    ADC_ExternalTrigConvCmd (ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}



/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  /* Enable DMA1 channel1 IRQ Channel (PSD�p���荞��)*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM4 Interrupt (�X�C�b�`�p���荞��)*/
   NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

/**PSD���m�p
  * @brief  Configure TIM1
  * @param  None
  * @retval : None
  */
void TIM1_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Supply APB2 Clock
  RCC_APB2PeriphClockCmd(TIM1_RCC , ENABLE);

#if defined (PARTIAL_REMAP_TIM1)
PartialRemap_TIM1_Configuration();
#elif defined (FULL_REMAP_TIM1)
FullRemap_TIM1_Configuration();
#endif

  /* ---------------------------------------------------------------------------
    TIM1 Configuration: Output Compare Toggle Mode:
    TIM1CLK = 72 MHz, Prescaler = 36000, TIM1 counter clock = 2kHz
  ----------------------------------------------------------------------------*/

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ARR_Val;
  TIM_TimeBaseStructure.TIM_Prescaler = 36000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Output Compare Toggle Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

  /* TIM enable counter */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/**�X�C�b�`�p
  * @brief  Configure TIM4
  * @param  None
  * @retval : None
  */
void TIM4_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  //Supply APB1 Clock��TIM4��
  RCC_APB1PeriphClockCmd(TIM4_RCC , ENABLE);

  /* ---------------------------------------------------------------------------
    TIM4 Configuration: Output Compare Toggle Mode:
    TIM4CLK = 36 MHz, Prescaler = 36000, TIM4 counter clock = 1kHz
  ----------------------------------------------------------------------------*/

  /* Time base configuration (TIM4�p)*/
  TIM_TimeBaseStructure.TIM_Period = SAMPLE_INTERVAL;
  TIM_TimeBaseStructure.TIM_Prescaler = 36000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* TIM IT enable */
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

}
//PWM�o�͗pTIM3
void TIM3_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Supply APB1 Clock
  RCC_APB1PeriphClockCmd(TIM3_RCC , ENABLE);
  //Supply APB2 Clock
  RCC_APB2PeriphClockCmd(TIM3_CH12_GPIO_RCC | TIM3_CH34_GPIO_RCC , ENABLE);

  /* GPIO Configuration:TIM3 Channel1 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = TIM3_CH1_PIN | TIM3_CH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TIM3_CH12_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = TIM3_CH3_PIN | TIM3_CH4_PIN ;
  GPIO_Init(TIM3_CH34_PORT, &GPIO_InitStructure);

#if defined (PARTIAL_REMAP_TIM3)
PartialRemap_TIM3_Configuration();
#elif defined (FULL_REMAP_TIM3)
FullRemap_TIM3_Configuration();
#endif

  /* ---------------------------------------------------------------------------
    TIM3 Configuration: Output Compare Toggle Mode:
    TIM3CLK = 36 MHz, Prescaler = 3600, TIM3 counter clock = 10KHz
  ----------------------------------------------------------------------------*/

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 100;
  TIM_TimeBaseStructure.TIM_Prescaler = 3600;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Output Compare Toggle Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/**�X�C�b�`���荞�݌��m�v���O����
  * @brief  Scan state of switches
  * @param  None
  * @retval : None
  */
void Scan_Switch(void)
{
  //***************
  //PA4�̓��̓`�F�b�N
  SW0_present_state = GPIO_ReadInputDataBit(GPIOY_4_PORT,GPIOY_4_PIN);
  if (SW0_present_state != SW0_previous_state)
    {
      SW0_scan_count++;
      if (SW0_scan_count > SAMPLE_TIMES)
        {
          SW0_previous_state = SW0_present_state;
          SW0_flag = SET;
        }
    }
  else
    {
      SW0_scan_count = 0;
    }
  //PA5�̓��̓`�F�b�N
  SW1_present_state = GPIO_ReadInputDataBit(GPIOY_5_PORT,GPIOY_5_PIN);
    if (SW1_present_state != SW1_previous_state)
      {
        SW1_scan_count++;
        if (SW1_scan_count > SAMPLE_TIMES)
          {
            SW1_previous_state = SW1_present_state;
            SW1_flag = SET;
          }
      }
    else
      {
        SW1_scan_count = 0;
      }

 ///*******************
    //
  if (SW0_flag != RESET && SW0_present_state == 0)
    {
      SW0_flag = RESET;
      SW0_is_released();
    }
  else if (SW0_flag != RESET && SW0_present_state != 0)
    {
      SW0_flag = RESET;
      SW0_is_pressed();
    }
  if (SW1_flag != RESET && SW1_present_state == 0)
      {
        SW1_flag = RESET;
        SW1_is_released();
      }
    else if (SW1_flag != RESET && SW1_present_state != 0)
      {
        SW1_flag = RESET;
        SW1_is_pressed();
      }
}
/**PA4�������ꂽ�Ƃ��̔����֐�
  * @brief  Action when SW0 is released
  * @param  None
  * @retval : None
  */
void SW0_is_released(void)
{
  //cprintf("PA4 is released. \r\n");
}

/**PA4�������ꂽ���̔����֐�
  * @brief  Action when SW0 is pressed
  * @param  None
  * @retval : None
  */
void SW0_is_pressed(void)
{
  //cprintf("PA4 is pressed. \r\n");
  XBee_Port_Send_String("b@");
	back();
	delay_ms(500);
	turnleft();
  }

/**PA5�������ꂽ�Ƃ��̔����֐�
  * @brief  Action when SW1 is released
  * @param  None
  * @retval : None
  */
void SW1_is_released(void)
{
  //cprintf("PA5 is released. \r\n");
}

/**PA5�������ꂽ���̔����֐�
  * @brief  Action when SW1 is pressed
  * @param  None
  * @retval : None
  */
void SW1_is_pressed(void)
{
  //cprintf("PA5 is pressed. \r\n");
  XBee_Port_Send_String("b@");
	back();
	delay_ms(500);
	turnleft();
}

//PSD�Z���T���m���荞�݃v���O����
void DMA1_Channel1_IRQHandler(void)
{
  /* Test on DMA1 Channel1 Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
 /*PA0(��)�̌��o*/
      ADC_Value1 = ADCConvertedValue[0];
      if(ADC_Value1 < Red_zone-100)
           {
          count1[0]++;
          count1[1] = 0;
          count1[2] = 0;
           }
      else if (ADC_Value1 > Red_zone)
              {
                count1[0] = 0;
                count1[1] = 0;
                count1[2]++;
              }
      if (count1[0] > 0 && State1 != 0)
             {
               State1 = 0;
               //cprintf("Green1 zone.\r\n");
               GPIO_ResetBits(GPIOX_PORT, GPIOX_4_PIN);
             }
      if (count1[2] > 0 && State1 != 2)
              {
                State1 = 2;
                XBee_Port_Send_String("2@");
                //cprintf("Red1 zone.\r\n");
                GPIO_SetBits(GPIOX_PORT, GPIOX_4_PIN);
//                turnleft();
              }
      /*PA1(�O)�̌��o*/
      ADC_Value2 = ADCConvertedValue[1];
      if(ADC_Value2 < Red_zone-100)
                 {
                count2[0]++;
                count2[1] = 0;
                count2[2] = 0;
                 }
      else if (ADC_Value2 > Red_zone)
                 {
                count2[0] = 0;
                count2[1] = 0;
                count2[2]++;
                 }
      if (count2[0] > 0 && State2 != 0)
                 {
                 State2 = 0;
                 //cprintf("Green2 zone.\r\n");
                 GPIO_ResetBits(GPIOX_PORT, GPIOX_3_PIN);
                 }
      if (count2[2] > 0 && State2 != 2)
                 {
                 State2 = 2;
                 XBee_Port_Send_String("1@");
                 //cprintf("Red2 zone.\r\n");
                 GPIO_SetBits(GPIOX_PORT, GPIOX_3_PIN);
                 /*stop();
                 delay_ms(3000);
                 */
                 }

      /*PA2�i�E�j�̌��o*/
      ADC_Value3 = ADCConvertedValue[2];
      if(ADC_Value3 < Red_zone-100)
                 {
                 count3[0]++;
                 count3[1] = 0;
                 count3[2] = 0;
                 }
      else if (ADC_Value3 > Red_zone)
                 {
                 count3[0] = 0;
                 count3[1] = 0;
                 count3[2]++;
                 }
      if (count3[0] > 0 && State3 != 0)
                 {
                 State3 = 0;
                 //cprintf("Green3 zone.\r\n");
                 GPIO_ResetBits(GPIOX_PORT, GPIOX_5_PIN);
                 }
      if (count3[2] > 0 && State3 != 2)
                 {
                 State3 = 2;
                 XBee_Port_Send_String("3@");
                 //cprintf("Red3 zone.\r\n");
                 GPIO_SetBits(GPIOX_PORT, GPIOX_5_PIN);
//                 turnright();
                 }
      /*PA3(���)�̌��o*/
                 ADC_Value4 = ADCConvertedValue[3];
                 if(ADC_Value4 < Red_zone-100)
                 {
                 count4[0]++;
                 count4[1] = 0;
                 count4[2] = 0;
                 }
                 else if (ADC_Value4 > Red_zone)
                {
                count4[0] = 0;
                count4[1] = 0;
                count4[2]++;
                }
                if (count4[0] > 0 && State4 != 0)
                {
                State4 = 0;
                //cprintf("Green4 zone.\r\n");
                GPIO_ResetBits(GPIOX_PORT, GPIOX_6_PIN);
                }
                if (count4[2] > 0 && State4 != 2)
                {
                State4 = 2;
                XBee_Port_Send_String("4@");
                //cprintf("Red4 zone.\r\n");
                GPIO_SetBits(GPIOX_PORT, GPIOX_6_PIN);
//                rotate();
                }


         /* Clear DMA1 Channel1 Transfer Complete interrupt pending bits */
         DMA_ClearITPendingBit(DMA1_IT_TC1);
         /* Clear ADC1 EOC pending interrupt bit */
               ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
       }
}
//�O�i�֐�
void forward(void)
{
  XBee_Port_Send_String("forward@");
  TIM_SetCompare3(TIM3,0);
  TIM_SetCompare4(TIM3,100);
  delay_ms(forwardtime);
  TIM_SetCompare3(TIM3,50);
  TIM_SetCompare4(TIM3,50);

  TIM_SetCompare1(TIM3,50);
}
//��ފ֐�
void back(void)
{
  XBee_Port_Send_String("back@");
  TIM_SetCompare3(TIM3,100);
  TIM_SetCompare4(TIM3,0);
  delay_ms(backtime);
  TIM_SetCompare3(TIM3,50);
  TIM_SetCompare4(TIM3,50);
}
//�E��]�֐�
void turnright(void)
{
  XBee_Port_Send_String("t@");
	TIM_SetCompare3(TIM3,0);
	TIM_SetCompare4(TIM3,0);
    delay_ms(rightangle);
	TIM_SetCompare3(TIM3,50);
	TIM_SetCompare4(TIM3,50);

}
//����]�֐�
void turnleft(void)
{
  XBee_Port_Send_String("t@");
  TIM_SetCompare3(TIM3,100);
  TIM_SetCompare4(TIM3,100);
  delay_ms(leftangle);
  TIM_SetCompare3(TIM3,50);
  TIM_SetCompare4(TIM3,50);
}
//���]�֐�
void rotate(void)
{
  XBee_Port_Send_String("t@");
	TIM_SetCompare3(TIM3,100);
	TIM_SetCompare4(TIM3,100);
    delay_ms(rotateangle);
	TIM_SetCompare3(TIM3,50);
	TIM_SetCompare4(TIM3,50);
}
//�e���ˊ֐�
void gun(void)
{
  TIM_SetCompare1(TIM3,90);
  delay_ms(guntime);
  TIM_SetCompare1(TIM3,50);
  XBee_Port_Send_String("l@");
}
//�Ђ��Ȃ��֐�
void bend(void)
{
  TIM_SetCompare2(TIM3,100);
  delay_ms(500);
  TIM_SetCompare2(TIM3,50);
}
//�Ђ��L�΂��֐�
void stretch(void)
{
  TIM_SetCompare2(TIM3,0);
  delay_ms(500);
  TIM_SetCompare2(TIM3,50);
}
//���Ɏw�肳�ꂽ�p�x(searchangle)������]
void turnsearch(void)
{
	TIM_SetCompare3(TIM3,searchspeed);
	TIM_SetCompare4(TIM3,searchspeed);
	   delay_ms(searchangle);
	TIM_SetCompare3(TIM3,50);
	TIM_SetCompare4(TIM3,50);
}
//stop
void stop(void)
{
  TIM_SetCompare1(TIM3,50);
  TIM_SetCompare2(TIM3,50);
  TIM_SetCompare3(TIM3,50);
  TIM_SetCompare4(TIM3,50);
}

