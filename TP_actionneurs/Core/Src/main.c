/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 1
#define CMD_BUFFER_SIZE 64
#define MAX_ARGS 9
// LF = line feed, saut de ligne
#define ASCII_LF 0x0A
// CR = carriage return, retour chariot
#define ASCII_CR 0x0D
// DEL = delete
#define ASCII_DEL 0x7F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t prompt[]="user@Nucleo-STM32G431>>";
const uint8_t started[]=
		"\r\n*-----------------------------*"
		"\r\n| Welcome on Nucleo-STM32G431 |"
		"\r\n*-----------------------------*"
		"\r\n";
const uint8_t newLine[]="\r\n";
const uint8_t cmdNotFound[]="Command not found\r\n";
const uint8_t help[]=
		"\r\n*-----------------------------*"
		"\r\n| Help Menu |"
		"\r\n*-----------------------------*"
		"\r\n*Set PA5 1 : Turns ON the LED*"
		"\r\n*Set PA5 0 : Turns OFF the LED*"
		"\r\n*ETCCC*"
		"\r\n";

const uint8_t pinout[]=
		"\r\n*-----------------------------*"
		"\r\n| Pinout List |"
		"\r\n*-----------------------------*"
		"\r\n*PA5: LED*"
		"\r\n*ETCCC*"
		"\r\n";
const uint8_t powerOn[]=
		"\r\n*-----------------------------*"
		"\r\n| Motor ON |"
		"\r\n";
const uint8_t powerOff[]=
		"\r\n*-----------------------------*"
		"\r\n| Motor OFF |"
		"\r\n";

uint32_t uartRxReceived;
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	char	 	cmd[CMD_BUFFER_SIZE];
	int 		idxCmd;
	char* 		argv[MAX_ARGS];
	int		 	argc = 0;
	char*		token;
	int 		newCmdReady = 0;

	/*
	const uint8_t help[] : contenant le message d'aide, la liste des fonctions
	const uint8_t pinout[] : contenant la liste des pin utilisées
	const uint8_t powerOn[] : contenant le message d'allumage du moteur
	const uint8_t powerOff[] : contenant le message d'extinction du moteur
	 */

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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	memset(argv,NULL,MAX_ARGS*sizeof(char*));
	memset(cmd,NULL,CMD_BUFFER_SIZE*sizeof(char));
	memset(uartRxBuffer,NULL,UART_RX_BUFFER_SIZE*sizeof(char));
	memset(uartTxBuffer,NULL,UART_TX_BUFFER_SIZE*sizeof(char));

	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2, started, sizeof(started), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, prompt, sizeof(prompt), HAL_MAX_DELAY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// uartRxReceived is set to 1 when a new character is received on uart 2
		if(uartRxReceived){
			switch(uartRxBuffer[0]){
			// Nouvelle ligne, instruction à traiter
			case ASCII_CR:
				HAL_UART_Transmit(&huart2, newLine, sizeof(newLine), HAL_MAX_DELAY);
				cmd[idxCmd] = '\0';
				argc = 0;
				token = strtok(cmd, " ");
				while(token!=NULL){
					argv[argc++] = token;
					token = strtok(NULL, " ");
				}

				idxCmd = 0;
				newCmdReady = 1;
				break;
				// Suppression du dernier caractère
			case ASCII_DEL:
				cmd[idxCmd--] = '\0';
				HAL_UART_Transmit(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
				break;
				// Nouveau caractère
			default:
				cmd[idxCmd++] = uartRxBuffer[0];
				HAL_UART_Transmit(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
			}
			uartRxReceived = 0;
		}


		if(newCmdReady){
			if(strcmp(argv[0],"set")==0){
				if(strcmp(argv[1],"PA5")==0){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, atoi(argv[2]));
					sprintf(uartTxBuffer,"Switch on/off led : %d\r\n",atoi(argv[2]));
					HAL_UART_Transmit(&huart2, uartTxBuffer, 32, HAL_MAX_DELAY);
				}
				else{
					HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
				}
			}
			else if(strcmp(argv[0],"get")==0)
			{
				HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"help")==0)
			{
				HAL_UART_Transmit(&huart2, help, sizeof(help), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"pinout")==0)
			{
				HAL_UART_Transmit(&huart2, pinout, sizeof(pinout), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"start")==0)
			{
				HAL_UART_Transmit(&huart2, powerOn, sizeof(powerOn), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"stop")==0)
			{
				HAL_UART_Transmit(&huart2, powerOff, sizeof(powerOff), HAL_MAX_DELAY);
			}
			else{
				HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
			}
			HAL_UART_Transmit(&huart2, prompt, sizeof(prompt), HAL_MAX_DELAY);
			newCmdReady = 0;
		}
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){
	uartRxReceived = 1;
	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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