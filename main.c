/** 
  ******************************************************************************
  * @file    main.c
  * @brief   Firmware for Silent Generator Micro SO8 "Comparator"
  *           - controlls one RGBW LED as battery meter
  *           - reads ADC value from battery voltage
  *           - reads ADC value for Power LED brightness
  *           - PWM control of white Power LED
  * @author  simon.burkhardt@eta-systems.ch
  * @version 0.3
  * @date    2021-04-09
  * 
  * @note    stm8s001j3Mx
  * @note    IAR Embedded Workbench for STM8 v3.11.1
  *          PC-locked license
  *          IAR for STMicroelectronics STM8, 8K KickStart Edition 3.11
  * 
  * @note    see CubeMX PDF
  *
  * @note    Code Size: 5'457 bytes (approximately)
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

#define ADC_CHANNEL_VBAT  ADC1_CHANNEL_6
#define ADC_CHANNEL_DIMM  ADC1_CHANNEL_5

#define VAL_RED_TO_GREEN  4
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static u32 uptime = 0;     // for delay routine
static u32 delay_time = 0;
volatile unsigned int adcVbat = 0;
volatile unsigned int adcDimm = 0;
u16 brightness = 0;
BitStatus swLED = RESET;
bool underVoltage = FALSE;

/* Private function prototypes -----------------------------------------------*/
void CLK_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void _delay_ms(u16);
void uptime_routine(void);
u16 ADC1_ReadVal(ADC1_Channel_TypeDef);

/* Private user code ---------------------------------------------------------*/
void main( void )
{  
  /* MCU Configuration -------------------------------------------------------*/
  CLK_Config();
  TIM2_Config();
  TIM4_Config();
  GPIO_Config();
  
  enableInterrupts();
  
  TIM2_SetCompare3(0);  // Power LED PWM value (0 - 1023);
  
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
    
    /* Switch and ADC polling */
    adcVbat = ADC1_ReadVal( ADC_CHANNEL_VBAT );
    adcDimm = ADC1_ReadVal( ADC_CHANNEL_DIMM );
    swLED = GPIO_ReadInputPin(GPIOB, GPIO_PIN_4);
    
    /* RGB LED BATTERY STATUS */
    /** @todo Undervoltage Lockout for Power LED */
    /** @todo Hysteresis when charging */
    // calculate Battery charge state
    // R-divider of 100k / 47k --> 47/147 = 0.319 = 1/3.127
    // Li-Ion fully charged = 8.5V --> 2.717V --> 556 ADC Code
    // Li-Ion empty = 3.6V --> 1.15V --> 235 ADC Code
    // --> above 480 = green
    // --> in between = orange
    // --> below 350 = red
    underVoltage = (adcVbat < 235) ? TRUE : FALSE;
    
    if(underVoltage){
      LED51.R = 10;
    } else {
      LED51.W = 0;
      if(adcVbat > 480){
        LED51.R = 0;
        LED51.G = 50;
      } else if (adcVbat > 350) {
        LED51.R = 20;
        LED51.G = 10;
      } else {
        LED51.R = 50;
        LED51.G = 0;
      }
    }
    
    /* POWER LED PWM */
    brightness = (adcDimm > 1023) ? 1023 : adcDimm;  // should only be a 12 Bit value
    brightness /= 4;  // convert to 8 Bit value
    // @note: swLED is active LOW --> RESET = turn on
    // (the swLED actually also turns on the DC supply for the STM8)
    if(swLED == RESET){
      // LED51.W = brightness / 4;
      TIM2_SetCompare3(brightness * 4);
    }else{
      // LED51.W = 0;
      TIM2_SetCompare3(0);
    }
    
    rgb_SetColor(0, LED51);
    rgb_SendArray();  // update LEDs
    
    /** @todo send to deep-sleep for ca. 10 - 100 ms */
    _delay_ms(10);
  }
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
  GPIO_Init (GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST); // Pin 5 PA3 OUT - LED PWM
  GPIO_Init (GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);     // Pin 6 PB4 IN  - LED On Switch
  GPIO_Init (GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST); // Pin 7 PC5 OUT - RGB_LED RGBW Pulse Data
  
  GPIO_Init (GPIOD, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);     // Pin 8 PD5 ADC - LED Dimmer Value
  GPIO_Init (GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);     // Pin 1 PD6 ADC - Battery Voltage (Vbat)
  
}

/* ADC1 Read -----------------------------------------------------------------*/
/**
  * @brief  Configure ADC1 channel, read several values and return average
  * @param  ch the ADC1 channel to sample
  * @retval 12 Bit ADC value
  * @note   https://circuitdigest.com/microcontroller-projects/adc-on-stm8s-using-c-compiler-reading-multiple-adc-values-and-displaying-on-lcd
  */
u16 ADC1_ReadVal(ADC1_Channel_TypeDef ch)
{
  u16 v[4] = {0,0,0,0};
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
             ch,
             ADC1_PRESSEL_FCPU_D18,
             ADC1_EXTTRIG_TIM,
             DISABLE,
             ADC1_ALIGN_RIGHT,
             ADC1_SCHMITTTRIG_ALL,
             DISABLE);
  
  ADC1_Cmd(ENABLE);
  ADC1_StartConversion();
  
  for(u8 i=0; i<4; i++){
    while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
    v[i] = ADC1_GetConversionValue();
    ADC1_ClearFlag(ADC1_FLAG_EOC);
  }
  ADC1_DeInit();
  
  // averaging
  u32 val = 0;
  for(u8 i=0; i<4; i++){
    val += v[i];
  }
  return (u16)(val/4);
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

/* Uptime Routine ------------------------------------------------------------*/
/* called from TIM4 interrupt in stm8s_it.c */
void uptime_routine(void)
{
  uptime++;
  if(uptime == 0xFFFFFFFF){
    delay_time = 0;
    uptime  = 0;
  }
}

/* Delay Routine -------------------------------------------------------------*/
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

