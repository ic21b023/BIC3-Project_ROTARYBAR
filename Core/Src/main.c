/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mylibrary.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RxBuffer_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MY_BARGRAPH_HandleTypeDef hbar1;

MY_ROTARY_HandleTypeDef hrot1;

SERIALPROTOCOL_TypeDef hserialprot;


uint32_t debounce_time_start = 10;
uint8_t buttonclicks=0;

uint16_t last_volume=0;
uint16_t mute_volume=8;
uint16_t bargraph_volume=0;
uint16_t rotary_volume=0;

uint8_t bargraph_brightness=20;
uint8_t seperate = 0;

uint16_t ring_rotation_led[2]={0b1100110011001100,
					0b0011001100110011

};

// Deklarieren und initialisieren des Ein-Zeichen-Empfangsbuffers
uint8_t RxBuffer[RxBuffer_SIZE]={0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t calculate_bargraph_bitframe(uint8_t pos);
uint8_t * calculate_rotary_brightness(uint16_t pos);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
 {

	 for (int i = 0; i < len; i++)
	 {
		 ITM_SendChar((*ptr++));
	 }
	 return len;
 }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, RxBuffer, RxBuffer_SIZE);


  MY_ROTARY_Init_Strobed(&hrot1,&htim6,MOSI_GPIO_Port, MOSI_Pin,MISO_GPIO_Port,MISO_Pin, SCK_GPIO_Port, SCK_Pin, CS_2_GPIO_Port, CS_2_Pin,ENC_A_GPIO_Port,ENC_A_Pin,ENC_B_GPIO_Port,ENC_B_Pin,SWITCH_GPIO_Port,SWITCH_Pin);
  MY_BARGRAPH_Init_Pulsed(&hbar1,&htim1,TIM_CHANNEL_3,MOSI_GPIO_Port, MOSI_Pin,MISO_GPIO_Port,MISO_Pin, SCK_GPIO_Port, SCK_Pin, CS_1_GPIO_Port, CS_1_Pin);


  /* Segmente aufgrund der Lautstärke ändern */
  uint16_t calculate_bargraph_bitframe(uint8_t pos){
	uint16_t bitframe=0;
    for (uint8_t i=0;i<pos/8;i++){
    	bitframe |=(1<<(i));
    }
    return bitframe;
  }

  /* Helligkeit aufgrund der Lautstärke ändern */
  uint8_t * calculate_rotary_brightness(uint16_t pos){
	  static uint8_t rotaty_brightness[16]={0};
	  for(uint8_t i=0; i<16;i++){
		rotaty_brightness[i]=(pos/8)*10;
	  }
	return rotaty_brightness;
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Variable zum koppeln und entkoppeln der Module */
	  if(!seperate){
		  bargraph_volume=rotary_volume;
	  }

	/* kalkulierte LED's des Encoders Ein- bzw. Ausschalten und die Helligkeit einstellen  */
	MY_ROTARY_SET_LEDS(&hrot1, ring_rotation_led[rotary_volume%2], calculate_rotary_brightness(rotary_volume));

	/* kalkulierte Segmente des Bargraphen Ein- bzw. Ausschalten und die Helligkeit einstellen */
	MY_BARGRAPH_SET_BITS(&hbar1, calculate_bargraph_bitframe(bargraph_volume), bargraph_brightness);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 40000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_1_Pin|SCK_Pin|MOSI_Pin|CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC_B_Pin ENC_A_Pin */
  GPIO_InitStruct.Pin = ENC_B_Pin|ENC_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_Pin */
  GPIO_InitStruct.Pin = SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_1_Pin SCK_Pin MOSI_Pin CS_2_Pin */
  GPIO_InitStruct.Pin = CS_1_Pin|SCK_Pin|MOSI_Pin|CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	// Alle Interrupts deaktivieren -> keine Nested-Interrupts
	__disable_irq();

	// Encoder entprellen
	if((HAL_GetTick() - debounce_time_start) > 60)
	{

		// Ermitteln in welche Richtung der Encoder gedreht wurde bzw. ob der Taster gedrückt wurde
		// 1=rechtsdreh; 2= linksdreh; 3=taster gedrückt
		uint8_t event = MY_ROTARY_GetEncoderEvent();

		// Wenn Encoder nach rechts gedreht wurde
		if(event==1){
			debounce_time_start =HAL_GetTick();
			last_volume=rotary_volume;

			// Buttonclicks zurücksetzen
			if(buttonclicks==1){
				buttonclicks=0;
			}

			// Lautstärke erhöhen -> Begrenzung bei 87
			if( rotary_volume <87){
					rotary_volume++;
			}

		// Wenn Encoder nach links gedreht wurde
		}else if(event==2){

			// Zeit des letzten Ereignisses speichern
			debounce_time_start =HAL_GetTick();

			last_volume=rotary_volume;

			// Buttonclicks zurücksetzen
			if(buttonclicks==1){
				buttonclicks=0;
			}

			// Lautstärke verringern -> Begrenzung bei 0
			if( rotary_volume >0){
					rotary_volume--;
			}

		// Wenn Encoder-Taster gedrückt wurde
		}else if(event==3){
			buttonclicks++;

			/* Wenn Rotarybutton 1x geklickt -> MUTE aktivieren -> funktioniert nur wenn die aktuelle Läutstärke höher ist als die Mute-Lautstärke */
			if(buttonclicks==1){
				if(rotary_volume>mute_volume){
					last_volume=rotary_volume;
					rotary_volume=mute_volume;
				}
			}

			/* Wenn Rotarybutton 1x geklickt -> MUTE beenden */
			if(buttonclicks==2){
				rotary_volume=last_volume;
				buttonclicks=0;
			}
		}

	}

	// Alle Interrupts aktivieren
	__enable_irq();
}

/* UART-Callback wird nach jedem Zeichen aufgerufen */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Nachrichtenbuffer für die Konsolen-Nachricht */
	uint8_t exchangedMessage[55] ={0};

	/*
	 * MYLIB_SERIALPROT_XCHANGE -> Verarbeitet die eingegebenen Zeichen des UART und gibt das demenstspechende Ergebnis/Nachricht zurück
	 * hserialprot -> Objekt des Seriellen Protokolls
	 * RxBuffer -> Ein-Zeichen-Empfangspuffer
	 * exchangedMessage -> zurückgegebene Nachricht aufgrund der Eingaben von Rx bzw. auf der Konsole
	 * Die Verarbeitung erfolgt in der MyLibrary/mylib_serialprot-Bibliothek
	 */
	MYLIB_SERIALPROT_XCHANGE(&hserialprot,RxBuffer,exchangedMessage);

	/* exchangedMessage an Putty/Konsole senden */
	if(HAL_UART_Transmit(&huart2, exchangedMessage,(uint16_t)strlen(exchangedMessage), 100)!= HAL_OK){Error_Handler();}

	/* UART_Receive Interrupt aktivieren */
	if(HAL_UART_Receive_IT(&huart2, RxBuffer, RxBuffer_SIZE)!= HAL_OK){Error_Handler();}
}

/* Callback für CRG-Commands (Rotary-Get), welche der User selbst definieren kann */
uint16_t SERIALPROT_Command_CRG_Callback(SERIALPROTOCOL_TypeDef *hserialprot)
{
	/* Prüfen ob das Kommando "crg" mit dem Parameter 1 "vol" eingegeben wurde und danach die Lautstärke zurückgeben */
	if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vol")){
		return rotary_volume;
	/* Prüfen ob das Kommando "crg" mit dem Parameter 1 "mva" eingegeben wurde und danach die Mute-Lautstärke zurückgeben */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mva")){
		return mute_volume;
	/* Prüfen ob das Kommando "crg" mit dem Parameter 1 "sep" eingegeben wurde und danach den Status zurückgeben ob die Module getrennt sind */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"sep")){
		return seperate;
	}
	return 0;
}

/* Callback für CRS-Commands (Rotary-Set), welche der User selbst definieren kann */
uint8_t SERIALPROT_Command_CRS_Callback(SERIALPROTOCOL_TypeDef *hserialprot)
{

	/* Prüfen ob das Kommando "crs" mit dem Parameter 1 "vol" eingegeben wurde und danach die Lautstärke nach Parameter 2 ändern */
	if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vol")){
		rotary_volume=atoi(hserialprot->Parameter2);
		last_volume=rotary_volume;
		return 0;

	/* Prüfen ob das Kommando "crs" mit dem Parameter 1 "mva" eingegeben wurde und danach die Mute-Lautstärke nach Parameter 2 ändern */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mva")){
		mute_volume=atoi(hserialprot->Parameter2);
		return 0;

		/* Prüfen ob das Kommando "crs" mit dem Parameter 1 "sep" eingegeben wurde und je nach Parameter 2 (0=nicht koppeln/1=koppeln) die Module koppeln oder nicht koppeln */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"sep")){
		if(__SERIALPROT_IS_PARAMETER2(hserialprot,"1")){
			seperate=1;
		}else if(__SERIALPROT_IS_PARAMETER2(hserialprot,"0")){
			seperate=0;
		}
		return 0;

	/* Prüfen ob das Kommando "crs" mit dem Parameter 1 "vmx" eingegeben wurde und danachdie Maximal-Lautstärke einstellenn */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vmx")){
		rotary_volume=87;
		last_volume=rotary_volume;
		return 0;

	/* Prüfen ob das Kommando "crs" mit dem Parameter 1 "mte" eingegeben wurde und je nach Parameter 2 (0=Aus/1=Ein) auf Mute-Lautstärke einstellen */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mte")){
		if(__SERIALPROT_IS_PARAMETER2(hserialprot,"1")){
			if(rotary_volume>mute_volume){
				last_volume=rotary_volume;
				rotary_volume=mute_volume;
			}
		}else if(__SERIALPROT_IS_PARAMETER2(hserialprot,"0")){
			rotary_volume=last_volume;
		}
		return 0;
	}
	return 1;
}

/* Callback für CBG-Commands (Bargraph-Get), welche der User selbst definieren kann */
uint16_t SERIALPROT_Command_CBG_Callback(SERIALPROTOCOL_TypeDef *hserialprot)
{
	/* Prüfen ob das Kommando "cbg" mit dem Parameter 1 "vol" eingegeben wurde und danach die Lautstärke zurückgeben */
	if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vol")){
		return bargraph_volume;

	/* Prüfen ob das Kommando "cbg" mit dem Parameter 1 "bgn" eingegeben wurde und danach die Helligkeit zurückgeben */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"bgn")){
		return bargraph_brightness;
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mva")){
		return mute_volume;
	/* Prüfen ob das Kommando "crg" mit dem Parameter 1 "sep" eingegeben wurde und danach den Status zurückgeben ob die Module getrennt sind */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"sep")){
		return seperate;
	}
	return 0;

}

/* Callback für CBS-Commands (Bargraph-Set), welche der User selbst definieren kann */
uint8_t SERIALPROT_Command_CBS_Callback(SERIALPROTOCOL_TypeDef *hserialprot)
{

	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "vol" eingegeben wurde und danach die Lautstärke nach Parameter 2 ändern */
	if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vol")){
		bargraph_volume=atoi(hserialprot->Parameter2);
		last_volume=bargraph_volume;
		return 0;
	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "vol" eingegeben wurde und danach die Helligkeit nach Parameter 2 ändern */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"bgn")){
		bargraph_brightness=atoi(hserialprot->Parameter2);
		return 0;
	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "mva" eingegeben wurde und danach die Mute-Lautstärke nach Parameter 2 ändern */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mva")){
		mute_volume=atoi(hserialprot->Parameter2);
		return 0;
	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "sep" eingegeben wurde und je nach Parameter 2 (0=nicht koppeln/1=koppeln) die Module koppeln oder nicht koppeln */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"sep")){
		if(__SERIALPROT_IS_PARAMETER2(hserialprot,"1")){
			seperate=1;
		}else if(__SERIALPROT_IS_PARAMETER2(hserialprot,"0")){
			seperate=0;
		}
		return 0;
	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "vmx" eingegeben wurde und danacf die Maximal-Lautstärke einstellenn */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"vmx")){
		bargraph_volume=87;
		last_volume=bargraph_volume;
		return 0;
	/* Prüfen ob das Kommando "cbs" mit dem Parameter 1 "mte" eingegeben wurde und je nach Parameter 2 (0=Aus/1=Ein) auf Mute-Lautstärke einstellen */
	}else if(__SERIALPROT_IS_PARAMETER1(hserialprot,"mte")){
		if(__SERIALPROT_IS_PARAMETER2(hserialprot,"1")){
			last_volume=bargraph_volume;
			bargraph_volume=mute_volume;
		}else if(__SERIALPROT_IS_PARAMETER2(hserialprot,"0")){
			bargraph_volume=last_volume;
		}
		return 0;
	}
	return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
