/** 
  ******************************************************************************
  * @file    main.c
  * @brief   Firmware for Silent Generator Micro SO8 "Comparator"
  *           - controlls one RGBW LED as battery meter
  *           - reads ADC value from battery voltage
  *           - reads ADC value for Power LED brightness
  *           - PWM control of white Power LED
  * @author  simon.burkhardt@eta-systems.ch
  * @version 0.1
  * @date    2021-03-06
  * 
  * @note    stm8s001j3
  * @note    IAR Embedded Workbench for STM8 v3.11.1
  * 
  ******************************************************************************
  * Copyright (c) 2021 eta Systems GmbH
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm8s_conf.h"
#include "ws2812B_fx.h"
#include "stm8s_adc1.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIM4_PERIOD       124
#define TIM2_PERIOD       1024
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// uint16_t ADCdata = 0;  // is updated in stm8s_it.c
volatile unsigned int ADC_Vbat, ADC_Dimm;
uint32_t cnt = 0;

/* Private function prototypes -----------------------------------------------*/
void CLK_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void _delay_ms(u16);
void uptime_routine(void);
void ADC_update(void);

/* Private user code ---------------------------------------------------------*/
void main( void )
{  
  /* MCU Configuration -------------------------------------------------------*/
  CLK_Config();
  TIM4_Config();
  GPIO_Config();
  ADC_Config();
  
  enableInterrupts();
  
  TIM2_SetCompare3(0);  // Power LED PWM value (0 - 1023);
  
  ADC_Vbat = 0;
  ADC_Dimm  = 0;
  u16 brightness = 0;
  
  // INOLUX RGBW LED
  RGBColor_t LED51;
  LED51.R = 0;
  LED51.G = 0;
  LED51.B = 0;
  LED51.W = 0;
  rgb_SetColor(0, LED51);
  rgb_SendArray();
  
  /* Infinite Loop -----------------------------------------------------------*/
  while(1)
  { 
    /** @note Test RGB Color Cycle "Glitzer" :))) */
    // rainbowCycle(10); 
    
    // ADC polling
    ADC_update();
    
    // Update Power LED PWM brightness
    brightness = (ADC_Dimm > 1023) ? 1023 : ADC_Dimm;
    // TIM2_SetCompare3(brightness);
    
    /** @todo Hysteresis when charging */
    // calculate Battery charge state
    // R-divider of 100k / 47k --> 47/147 = 0.319 = 1/3.127
    // Li-Ion fully charged = 8.5V --> 2.717V --> 556 ADC Code
    // Li-Ion empty = 3.6V --> 1.15V --> 235 ADC Code
    // --> above 480 = green
    // --> in between = orange
    // --> below 350 = red
    if(ADC_Vbat > 480){
      LED51.R = 0;
      LED51.G = 50;
    } else if (ADC_Vbat > 350) {
      LED51.R = 25;
      LED51.G = 25;
    } else {
      LED51.R = 50;
      LED51.G = 0;
    }
    rgb_SetColor(0, LED51);
    rgb_SendArray();  // update LEDs
    
    /** @todo send to deep sleep for ca. 100ms */
    _delay_ms(100);
    
    /*
    // TESTING
    // quickly fade 1st White LED 
    LED51.R = 0;
    LED51.W = ++cnt%48;
    rgb_SetColor(0, LED51);
    
    // represent ADC value as last Red LED 
    LED51.B = (uint8_t) (ADC_Vbat >> 2);
    LED51.R = (uint8_t) (ADC_Dimm >> 2);
    LED51.W = 0;
    rgb_SetColor(7, LED51);
    
    rgb_SendArray();  // update LEDs
    _delay_ms(100);
    */
    
  }
}

void ADC_update(void)
{
  // http://embedded-lab.com/blog/continuing-stm8-microcontroller-expedition/2/
  ADC1_ScanModeCmd(ENABLE);
  ADC1_StartConversion();
  while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
  ADC1_ClearFlag(ADC1_FLAG_EOC);
  ADC_Vbat = ADC1_GetBufferValue(0);
  ADC_Dimm = ADC1_GetBufferValue(1);
}

/**
  * @brief  Configure system clock to run at Maximum clock speed and output the 
  *         system clock on CCO pin
  * @param  None
  * @retval None
  */
void CLK_Config(void)
{
  CLK_DeInit();
  CLK_HSECmd(DISABLE);
  CLK_LSICmd(DISABLE);
  CLK_HSICmd(ENABLE);
  while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == RESET);
  CLK_ClockSwitchCmd(ENABLE);
  /* Configure the Fcpu to DIV1*/
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  /* Configure the HSI prescaler to the optimal value */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  /* Output Fcpu on CLK_CCO pin */
  CLK_CCOConfig(CLK_OUTPUT_CPU);
  /* Configure the system clock to use HSI clock source and to run at 16Mhz */
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  
  /* Enable ADC1 clock */
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
}

/* GPIO Init -----------------------------------------------------------------*/
void GPIO_Config(void)
{
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOD);
  GPIO_Init (GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST); // Pin 5 - PA3 - LED PWM
  GPIO_Init (GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST); // Pin 6 - PB4 - LED On
  GPIO_Init (GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST); // Pin 7 - PC5 - RGB_LED RGBW Pulse Data
  
  GPIO_Init (GPIOD, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);     // Pin 8 - PD5 - ADC Dim
  GPIO_Init (GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);     // Pin 8 - PD6 - ADC VBAT
  
}

/* ADC1 Init -----------------------------------------------------------------*/
/**
  * @brief  Configure ADC peripheral 
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  /* Initialise and configure ADC1 */
  ADC1_DeInit();
  
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
            ADC1_CHANNEL_5, 
            ADC1_PRESSEL_FCPU_D18, 
            ADC1_EXTTRIG_GPIO, 
            DISABLE, 
            ADC1_ALIGN_RIGHT, 
            ADC1_SCHMITTTRIG_CHANNEL5, 
            DISABLE ); 
  
   ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
             ADC1_CHANNEL_6,
             ADC1_PRESSEL_FCPU_D18,
             ADC1_EXTTRIG_GPIO,
             DISABLE,
             ADC1_ALIGN_RIGHT,
             ADC1_SCHMITTTRIG_CHANNEL6,
             DISABLE);

   //ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);  // Interrupt Mode not working 
   ADC1_ConversionConfig(ADC1_CONVERSIONMODE_CONTINUOUS, 
                         ((ADC1_Channel_TypeDef)(ADC1_CHANNEL_5 | ADC1_CHANNEL_6)), 
                         ADC1_ALIGN_RIGHT);
   ADC1_DataBufferCmd(ENABLE);
   ADC1_Cmd(ENABLE);
}

/* Timer4 Init ---------------------------------------------------------------*/
void TIM4_Config(void)
{
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
}

/* Timer2 Init ---------------------------------------------------------------*/
// http://embedded-lab.com/blog/starting-stm8-microcontrollers/20/
void TIM2_Config(void)
{
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_32, TIM2_PERIOD);
  // TIM2 CH3 on PA3
  TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, TIM2_PERIOD, 
               TIM2_OCPOLARITY_HIGH);
  TIM2_Cmd(ENABLE);
}

static u32 uptime = 0;
static u32 delay_time = 0;
u8 z = 0;

/* Uptime Routine ------------------------------------------------------------*/
/* called from TIM4 interrupt in stm8s_it.c */
void uptime_routine(void)
{
  uptime++;
  if(uptime == 0xFFFFFFFF){
    delay_time = 0;
    uptime  = 0;
  }
  z++;
  if(z == 250){
    z = 0;
    GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
  }
}

void _delay_ms(u16 wait)
{ 
  delay_time = uptime + wait;
  while(delay_time > uptime){}
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  
  while (1)
  {
    
  }
}
#endif


/******************************* (c) 2020 eta Systems GmbH *****END OF FILE****/

