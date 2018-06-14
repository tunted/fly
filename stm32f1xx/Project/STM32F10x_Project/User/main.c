/**
  ******************************************************************************
  * @file    ADC/3ADCs_DMA/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "includes.h" 
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_3ADCs_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private system constants --------------------------------------------------*/
static const float PID1_KP        = 0.1;
static const float PID1_KI        = 0.05;
static const float PID1_KD        = 0.001;

static const float PID2_KP        = 0.2;
static const float PID2_KI        = 0.003;
static const float PID2_KD        = 0.001;


/* Private system constants --------------------------------------------------*/
static const float HALL_SENSOR_CALIB_FACTOR       = 0.00042;
static const uint32_t COIL_DRIVER_PWM_FREQ        = 5000;
static const uint32_t SYSTEM_TICK_FREQ            = 1000;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

/* Private variables ---------------------------------------------------------*/
static systick_task_controller_t task10ms;

static systick_task_controller_t task5s;

/* Private variables ---------------------------------------------------------*/
static pid_controller_t pid_controller_1, pid_controller_2; 

/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC1ConvertedValue[4] = {0, 0, 0, 0};
static hall_sensor_t sensor1, sensor2, sensor3, sensor4;
static coil_driver_t coil_driver_1, coil_driver_2;

/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void ADC_DMA_Configuration(void);
static void TIM_PWM_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* ADC configuration with DMA support --------------------------------------*/
  ADC_DMA_Configuration();

  /* TIM configuration for pwm function --------------------------------------*/
  TIM_PWM_Configuration();

  /* User config for hall sensor ---------------------------------------------*/
  SysTick_Config(SystemCoreClock / SYSTEM_TICK_FREQ);

  /* User config for hall sensor ---------------------------------------------*/
  systick_add_task_controller(&task10ms, 10);
  systick_add_task_controller(&task5s, 5000);


  /* User config pid controller ----------------------------------------------*/
  pid_controller_init(&pid_controller_1, PID1_KP, PID1_KI, PID1_KD);
  pid_controller_init(&pid_controller_2, PID2_KP, PID2_KI, PID2_KD);

  /* User config for hall sensor ---------------------------------------------*/
  hall_sensor_init(&sensor1, HALL_SENSOR_CALIB_FACTOR);
  hall_sensor_init(&sensor2, HALL_SENSOR_CALIB_FACTOR);
  hall_sensor_init(&sensor3, HALL_SENSOR_CALIB_FACTOR);
  hall_sensor_init(&sensor4, HALL_SENSOR_CALIB_FACTOR);

  /* User config for coil driver---------------------------------------------*/
  coil_driver_init(&coil_driver_1, TIM1, 1, 2);
  coil_driver_init(&coil_driver_2, TIM1, 3, 4);
 
  while (1)
  {
    if (true == systick_check_task_enabled(&task10ms))
    {
      pid_controller_routine(&pid_controller_1, 0.01, 1);
      pid_controller_routine(&pid_controller_2, 0.01, 4);
    }

    if (true == systick_check_task_enabled(&task5s))
    {
      pid_controller_set_ref(&pid_controller_1, 5);
      pid_controller_set_ref(&pid_controller_2, 5);
    }

  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
static void RCC_Configuration(void)
{
  /* ADCCLK = PCLK2/4 */
  // RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
   
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and 
	GPIOA clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,   ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,   ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,   ENABLE);

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PA.02, PA.03 and PA.04 (ADC Channel2, ADC Channel3 and 
     ADC Channel4) as analog inputs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

static void ADC_DMA_Configuration(void){
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);

  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1ConvertedValue;
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
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 4;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_239Cycles5);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_239Cycles5);    

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);

  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);

  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}


static void TIM_PWM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* Compute the value to be set in ARR regiter to generate signal frequency at default*/
  uint16_t TimerPeriod = (SystemCoreClock / COIL_DRIVER_PWM_FREQ) - 1;


  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
