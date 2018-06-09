/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "DRV8323.h"
#include "usb.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;


TIM_OC_InitTypeDef sConfigOC;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void set_PWM_Duty(uint16_t PWM1_VALUE,uint16_t PWM2_VALUE,uint16_t PWM3_VALUE, uint16_t PWM_PERIOD);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int i, len;
uint8_t stat, buf[500] = "Hello this is direct print\r\n";
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED,ENABLE); // show that we have started initialisaition process

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  HAL_Delay(2);
  /*Configure DRV8323  */
  HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PIN,ENABLE);// Disable the DRV83238
  HAL_Delay(2);
  DRV8323_Config();
  HAL_Delay(2);
 /* if(DRV8323_Config()==1){
	  HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PIN,ENABLE);// Enable the DRV83238}
  }else HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PIN,DISABLE);// Disable the DRV83238
*/

  MX_TIM1_Init();
  //HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PIN,DISABLE);// Disable the DRV83238

	//HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, ENABLE);

  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED,DISABLE);// show that we have finished initialisaition process

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1)
  {

	  static uint16_t PULSE[3] = {200};
	  uint16_t DRV8323_Data[7] = {0};
  /* USER CODE END WHILE */
	    HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED); // show that we are in main loop

	    if(HAL_GPIO_ReadPin (DRV8323_FAULT_PIN_PORT,DRV8323_FAULT_PIN)==0)
	    {
	    	HAL_GPIO_WritePin(RED_LED_PORT, RED_LED,ENABLE);// show that there is an error
	    //	HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PORT,DISABLE);// DISABLE the DRV83238
	    }else
	    {
	    	HAL_GPIO_WritePin(RED_LED_PORT, RED_LED,DISABLE);
	    }
	   //DRV8323_writeSpi(ADR_GATE_DRV_HS,DRV8323regGateDrvHS);
	   //DRV8323_Config();
	   HAL_Delay(1);
	   // DRV8323_writeSpi(ADR_DRV_CTRL, DRV8323regDrvCtrl);

	   DRV8323_Data[0] = DRV8323_readSpi(ADR_FAULT_STAT);
	   DRV8323_Data[1] = DRV8323_readSpi(ADR_VGS_STAT);
	   DRV8323_Data[2] = DRV8323_readSpi(ADR_DRV_CTRL);
	   DRV8323_Data[3] = DRV8323_readSpi(ADR_GATE_DRV_HS);
	   DRV8323_Data[4] = DRV8323_readSpi(ADR_GATE_DRV_LS);
	   DRV8323_Data[5] = DRV8323_readSpi(ADR_OCP_CTRL);
	   DRV8323_Data[6] = DRV8323_readSpi(ADR_CSA_CTRL);

	   /*
	   DRV8323_Data[0] = DRV8323regDrvCtrl;
	   DRV8323_Data[1] = DRV8323regGateDrvHS;
	   DRV8323_Data[2] =DRV8323regGateDrvLS;
	   DRV8323_Data[3] = DRV8323regOcpCtrl;
	   DRV8323_Data[4] = DRV8323regCsaCtrl;
   */

	    uint16_t string =  	"---------------------- \n"
	    				"ADR_FAULT_STAT:0x%X \n"
	    			  	"ADR_VGS_STAT:0x%X \n"
	    				"ADR_DRV_CTRL:0x%X \n"
	    				"ADR_GATE_DRV_HS:0x%X \n"
	    				"ADR_GATE_DRV_LS:0x%X \n"
	    				"ADR_OCP_CTRL:0x%X \n"
	    				"ADR_CSA_CTRL:0x%X \n"
	    				"--------------------- \n";

	    HAL_Delay(800);

	 	len=sprintf(buf,string, DRV8323_Data[0],DRV8323_Data[1],DRV8323_Data[2],DRV8323_Data[3],DRV8323_Data[4],DRV8323_Data[5],DRV8323_Data[6]);
	  	// n=sprintf(buffer,"ADR_OCP_CTRL:0x%X\n", DRV8323_Data[1]);
	  	CDC_Transmit_FS(buf, len);
	    /* Insert a 80ms delay */

	  	PULSE[0]= PULSE[0]+ 1;
	  	PULSE[1]= PULSE[0];
		PULSE[2]= PULSE[0];
		if(PULSE[0]>800) PULSE[0]=200;
		if(PULSE[1]>800) PULSE[1]=200;
		if(PULSE[2]>800) PULSE[2]=200;

	  	set_PWM_Duty(PULSE[0],PULSE[1],PULSE[2],PERIOD_VALUE);

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks  */

/*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 15;//15
  RCC_OscInitStruct.PLL.PLLN = 360;//360
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;// RCC_PLLP_DIV2
  RCC_OscInitStruct.PLL.PLLQ = 8;//8
  RCC_OscInitStruct.PLL.PLLR = 2;//2
  */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 6;


  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;//RCC_HCLK_DIV1

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  PeriphClkInitStruct.PLLSAI.PLLSAIN= 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIP= 4;


  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT; //acts as master//SPI_NSS_SOFT;//
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 10;//10

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

  /* Compute the prescaler value to have TIM1 counter clock equal to 18MHz */
 uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock  / 18000000) - 1);

  	  /*##-1- Configure the TIM peripheral #######################################*/
  	  /* ---------------------------------------------------------------------------
  	  1/ Generate 3 complementary PWM signals with 3 different duty cycles:

  	    TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2), since APB2
  	    prescaler is different from 1.
  	    TIM1CLK = 2 * PCLK2
  	    PCLK1 = HCLK / 2
  	    => TIM1CLK = HCLK = SystemCoreClock

  	    TIM1CLK is fixed to SystemCoreClock, the TIM1 Prescaler is set to have
  	    TIM1 counter clock = 18MHz.

  	    The objective is to generate PWM signal at 10 KHz:
  	    - TIM1_Period = (TIM1 counter clock / 10000) - 1

  	    The Three Duty cycles are computed as the following description:

  	    The channel 1 duty cycle is set to 50% so channel 1N is set to 50%.
  	    The channel 2 duty cycle is set to 25% so channel 2N is set to 75%.
  	    The channel 3 duty cycle is set to 12.5% so channel 3N is set to 87.5%.

  	   The Timer pulse is calculated as follows:
  	     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100

  	  2/ Insert a dead time equal to (100/SystemCoreClock) us

  	  3/ Configure the break feature, active at High level, and using the automatic
  	     output enable feature

  	  4/ Use the Locking parameters level1.

  	    Note:
  	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
  	     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
  	     variable value. Otherwise, any configuration based on this variable will be incorrect.
  	     This variable is updated in three ways:
  	      1) by calling CMSIS function SystemCoreClockUpdate()
  	      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
  	      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  	  --------------------------------------------------------------------------- */

  	  /* Initialize TIM peripheral as follows:
  	       + Prescaler = (SystemCoreClock/18000000) - 1
  	       + Period = (1800 - 1)  (to have an output frequency equal to 10 KHz)
  	       + ClockDivision = 0
  	       + Counter direction = Up
  	  */

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PERIOD_VALUE;
  htim1.Init.ClockDivision =2;//0
  htim1.Init.RepetitionCounter =0 ;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
 {
   _Error_Handler(__FILE__, __LINE__);
 }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  //set_PWM_Duty(500,500,500,1000);

  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  /* Set the pulse value for channel 2 */
  sConfigOC.Pulse = 520;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }
  /* Set the pulse value for channel 3 */
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

  // ##-3- Start PWM signals generation ####################################### START ALL DOESNT WORK have to do each 1 individually
 	   /* Start channels ALL*/

	   if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
 	   {
 	     /* Starting Error*/
 	     Error_Handler();
 	   }
 	   /* Start channel ALLN*/
 	   if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
 	   {
 	    /*Starting Error*/
 	     Error_Handler();
 	   }

	   if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
 	   {
 	     /* Starting Error*/
 	     Error_Handler();
 	   }
 	   /* Start channel ALLN*/
 	   if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
 	   {
 	    /*Starting Error*/
 	     Error_Handler();
 	   }

	   if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
 	   {
 	     /* Starting Error*/
 	     Error_Handler();
 	   }
 	   /* Start channel ALLN*/
 	   if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
 	   {
 	    /*Starting Error*/
 	     Error_Handler();
 	   }




}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}


void set_PWM_Duty(uint16_t PWM1_VALUE,uint16_t PWM2_VALUE,uint16_t PWM3_VALUE, uint16_t PWM_PERIOD)
{

	  TIM_OC_InitTypeDef sConfigOC;

	 // if(((PWM1_VALUE < PWM_PERIOD)&&(PWM2_VALUE < PWM_PERIOD))&&(PWM3_VALUE < PWM_PERIOD))
	 // {
		  // need to put in some limiting stuff here to prevent OC faults or PWM values that are out of range.

		  /* Set the pulse value for channel 1 */
		  sConfigOC.Pulse = PWM1_VALUE;
		  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		  {
			_Error_Handler(__FILE__, __LINE__);
		  }

		  /* Set the pulse value for channel 2 */
		  sConfigOC.Pulse = PWM2_VALUE;
		  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		  {
			_Error_Handler(__FILE__, __LINE__);
		  }
		  /* Set the pulse value for channel 3 */
		  sConfigOC.Pulse = PWM3_VALUE;
		  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		  {
			_Error_Handler(__FILE__, __LINE__);
		  }
	 // }else
	 // {


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}



#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
