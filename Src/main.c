
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32l0538_discovery_epd.h"
#include "stm32l0538_discovery.h"
//#include "font20epd.c"
//#include "font16epd.c"
//#include "font12epd.c"
//#include "font8epd.c"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint16_t AdcDmaBuf[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
 
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void Automat(void);

void SwitchR1(uint8_t State);
void SwitchR2(uint8_t State);
void GPIO_init(void);
uint8_t ReadButton(void);
void getVoltage(void);

extern volatile long long CounterClock;

uint8_t DischargeState = 0;
uint8_t Key = 0;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
  double Voltage = 0.0;
  double VoltageR = 0.0;
  double Current = 0.0;
  double CalcCapacity = 0.0;
  uint8_t StringToDisp[20] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  int tmpInt1 = 0;                  // Get the integer (678).
  float tmpFrac = 0.0;      // Get fraction (0.0123).
  int tmpInt2 = 0;  // Turn into integer (123).
  
  

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(TIM21_IRQn);
  HAL_NVIC_EnableIRQ(TIM22_IRQn); 
  HAL_TIM_Base_Start(&htim2);
  
  BSP_EPD_Init();
  
  BSP_EPD_Clear(EPD_COLOR_WHITE);
  
  
  GPIO_init();
  
  
  HAL_ADC_Start_DMA(&hadc,(uint32_t *)&AdcDmaBuf[0], 2 );
//HAL_ADC_Start_DMA(&hadc,(uint32_t *)&AdcDmaBuf[0], 3 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    BSP_EPD_Clear(EPD_COLOR_WHITE);
    BSP_EPD_SetFont(&Font12);

          tmpInt1 = (int)Voltage;
          tmpFrac = Voltage - tmpInt1;
          tmpInt2 = (int)trunc(tmpFrac * 1000);
        
          sprintf((char*)StringToDisp,"BAT Voltage: %d.%03d V", tmpInt1, tmpInt2);
          BSP_EPD_DisplayStringAt(1, 8, StringToDisp, LEFT_MODE);
        
          tmpInt1 = (int)Current;
          tmpFrac = Current - tmpInt1;
          tmpInt2 = (int)trunc(tmpFrac * 10000);    
        
          sprintf((char*)StringToDisp,"BAT Current:%d.%04d A", tmpInt1, tmpInt2);
          BSP_EPD_DisplayStringAt(1, 5, StringToDisp, LEFT_MODE);
        
          tmpInt1 = (int)CalcCapacity;
          tmpFrac = CalcCapacity - tmpInt1;
          tmpInt2 = (int)trunc(tmpFrac * 1000);      
        
          sprintf((char*)StringToDisp,"Calc. Cap.:%d.%03d mA*h", tmpInt1, tmpInt2);
          BSP_EPD_DisplayStringAt(1, 1, StringToDisp, LEFT_MODE);

        Automat();
        
        BSP_EPD_RefreshDisplay();
        HAL_Delay(1000);   
        ReadButton();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = ENABLE;
  hadc.Init.Oversample.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc.Init.Oversample.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc.Init.Oversample.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 100;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/

/* USER CODE BEGIN 4 */

void GPIO_init(void){
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  __GPIOB_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
 // __GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  

}

void SwitchR1(uint8_t State){
 if (!State)
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);
 else
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);
}

void SwitchR2(uint8_t State){
 if (!State)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
 else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t ReadButton(void){
  Key = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  if(Key){
    DischargeState++;
    DischargeState %= 3;
  }
//  switch(Key){
 //   case GPIO_PIN_SET: 
 //     return 1;
 //   default: 
 //     return 0;
 // }
 return Key;
}

 void Automat(void){
 //static uint32_t Seconds = 0;
 static uint32_t AutoVariable = 0;
 //static uint32_t DischargeStartSeconds = 0;
  
 //Seconds = CounterClock/1000;

 
 BSP_EPD_SetFont(&Font16);
 
   switch (DischargeState){
      case 3:
         // sprintf((char*)StringToDisp,"NO RES CONNECTED");
          BSP_EPD_DisplayStringAt(1, 11, "FINISHED.", LEFT_MODE);
          SwitchR1(0); SwitchR2(0);
          
        break;  

      case 1: 
        BSP_EPD_DisplayStringAt(1, 11, "R=5 Ohm CONN.", LEFT_MODE);
         SwitchR1(1); SwitchR2(0);
         
          break;
          
      case 2: 
        BSP_EPD_DisplayStringAt(1, 11, "R=25 Ohm CONN.", LEFT_MODE);
         SwitchR1(0); SwitchR2(1);
         
          break;
          
      case 0:
         // sprintf((char*)StringToDisp,"NO RES CONNECTED");
          BSP_EPD_DisplayStringAt(1, 11, "NO RES CONNECTED", LEFT_MODE);
          SwitchR1(0); SwitchR2(0);
          
        break;  
       
    }
   getVoltage();
   switch(AutoVariable % 3){
      case 0:   
         break;
    }
 AutoVariable++;
 }

#define Coeff  (0.0000505)
#define Coeff2 1056
#define R1 5
#define R2 25
#define TH_Voltage1 1.05
#define TH_Voltage2 1.0

void getVoltage(void){
  switch (DischargeState){
    case 0:
      VoltageR = (AdcDmaBuf[0] - Coeff2);
      Voltage = VoltageR * Coeff;
     // Voltage -= ( AdcDmaBuf[1] - Coeff2) ;
      VoltageR *= Coeff;
      break; 
    case 1:
      VoltageR = (AdcDmaBuf[0] - Coeff2);
      Voltage = VoltageR * Coeff;
      VoltageR -= ( AdcDmaBuf[1] - Coeff2) ;
      VoltageR *= Coeff;
      Current = VoltageR/R1;
      if (Voltage < TH_Voltage1){
        if(Voltage > 0.05) DischargeState++;
      }
      CalcCapacity += (CounterClock*Current)/3600;
      break;
     case 2:
      VoltageR = (AdcDmaBuf[0] - Coeff2);
      Voltage = VoltageR * Coeff;
     // Voltage -= ( AdcDmaBuf[1] - Coeff2) ;
      VoltageR *= Coeff;
      Current = VoltageR/R2;
      if (Voltage < TH_Voltage2){
       if(Voltage > 0.05) DischargeState++;
      }
      CalcCapacity += (CounterClock*Current)/3600;
      break;   
      case 3:
      VoltageR = (AdcDmaBuf[0] - Coeff2);
      Voltage = VoltageR * Coeff;
     // Voltage -= ( AdcDmaBuf[1] - Coeff2) ;
      VoltageR *= Coeff;
      Current = 0;
     // CalcCapacity += (CounterClock*Current)/3600;
      break; 
  
      
  }
   
   CounterClock = 0.0; 
 


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
