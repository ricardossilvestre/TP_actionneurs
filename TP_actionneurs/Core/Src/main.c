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
#include "adc.h"
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
#define UART_TX_BUFFER_SIZE 64				///< Buffer size for transmission used in the shell communication
#define UART_RX_BUFFER_SIZE 1				///< Buffer size for reception used in the shell communication
#define CMD_BUFFER_SIZE 64					///< Buffer size for the shell command line
#define MAX_ARGS 9							///< Maximum number of arguments for a command in the shell script
					
#define ASCII_CR 0x0D						///< Defines CR, carriage return command in the shell
#define ASCII_DEL 0x7F						///< Defines DEL, delete command in the shell	

#define SPEED_MAX 512 						///< Arbitrary number representing out maximum value for the motor speed

#define TIMCLOCK   170000000				///< Definition of bus frequency for clock, useful to calculate time
#define PRESCALAR  1						///< Definition of prescaler clock, useful to calculate time
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t prompt[]="user@Nucleo-STM32G431>>";		///< Shell prompt text
const uint8_t started[]=								///< Startup message when shell is initialized
		"\r\n*-----------------------------*"
		"\r\n| Welcome on Nucleo-STM32G431 |"
		"\r\n*-----------------------------*"
		"\r\n";
const uint8_t newLine[]="\r\n";							///< Defines characters used to create a new line
const uint8_t cmdNotFound[]="Command not found\r\n";	///< Shell message for when a user input command is not implemented
const uint8_t help[]=									///< Shell message for when the user types "help"
		"\r\n*-----------------------------*"
		"\r\n| Help Menu |"
		"\r\n*-----------------------------*"
		"\r\n*set PA5 1 : Turns ON the LED*"
		"\r\n*set PA5 0 : Turns OFF the LED*"
		"\r\n*get : Prints the current and the frequency in the motor*"
		"\r\n*pinout : Prints the pinout list*"
		"\r\n*start : Starts the motor*"
		"\r\n*speed x : Sets the speed of the motor to x (-512 < x < 512)*"
		"\r\n";

const uint8_t pinout[]=									///< Shell message that lists the pinout list of the microcontroller
		"\r\n*-----------------------------*"
		"\r\n| Pinout List |"
		"\r\n*-----------------------------*"
		"\r\n*PA0: ADC_CURRENT*"		
		"\r\n*PA5: LED*"
		"\r\n*PA8: TIM1_CH1*"
		"\r\n*PA9: TIM1_CH2*"
		"\r\n*PA11: TIM1_CH1N*"
		"\r\n*PA12: TIM1_CH2N*"
		"\r\n*PC3: ISO_RESET*"
		"\r\n";
const uint8_t powerOn[]=								///< Shell message when powering on the motor
		"\r\n*-----------------------------*"
		"\r\n| Motor ON |"
		"\r\n";
const uint8_t powerOff[]=								///< Shell message when powering off the motor
		"\r\n*-----------------------------*"
		"\r\n| Motor OFF |"
		"\r\n";

const char * separators = " =";							///< List of separators used to parse the strings in shell

uint32_t uartRxReceived;								///< Set to 1 when a new character is received on uart 2
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];				///< A buffer to store the received data from UART
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];				///< A buffer to store the UART data to be transmitted


uint32_t IC_Val1 = 0;									///< First timer value
uint32_t IC_Val2 = 0;									///< Second timer value
uint32_t Difference = 0;								///< Difference of the two timer values, to be able to know the direction of the rotation

int Is_First_Captured = 0;								///< Set to 1 when the first rising edge is captured
int Is_First_Captured_2 = 0;							///< Set to 1 when the first rising edge is captured (version 2)

/* Measure Frequency */
float frequency = 0;									///< Declaration of variable to hold the frequency value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
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
	char	 	cmd[CMD_BUFFER_SIZE];		///< Initializes a cmd list to store the characters sent via shell
	int 		idxCmd;						///< Initializes an index used to parse through the shell characters
	char* 		argv[MAX_ARGS];				///< Initializes a variable to hold the arguments of a function
	int		 	argc = 0;					///< Initializes a variable to hold the argc value
	char*		token;						///< Initializes a variable to hold the tokens from strtok function
	int 		newCmdReady = 0;			///< Initializes a variable to verify if shell has finished user input
	int		speed = 0;						///< Initializes a variable to hold the speed value
	uint16_t  CCR1 = 5312;					///< Initializes a variable to hold the value in the compare register channel 1
	uint16_t  CCR2 = 5312;					///< Initializes a variable to hold the value in the compare register channel 2

	uint16_t AD_RES = 0;					///< Initializes a variable to hold the value form the ADC conversion for the current

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

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
				token = strtok(cmd, separators);
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
			// Traiter l'instruction correspondante
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
				// Read The ADC Conversion Result & Map It To PWM DutyCycle
				AD_RES = ((3300*HAL_ADC_GetValue(&hadc1)/4096)-2500)*12;
				sprintf(uartTxBuffer,"ADC = %d | Freq = %d \r\n",AD_RES,(int) frequency);
				HAL_UART_Transmit(&huart2, uartTxBuffer, sizeof(uartTxBuffer), HAL_MAX_DELAY);

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
				powerUpSequence();
				HAL_UART_Transmit(&huart2, powerOn, sizeof(powerOn), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"stop")==0)
			{
				HAL_UART_Transmit(&huart2, powerOff, sizeof(powerOff), HAL_MAX_DELAY);
			}
			else if(strcmp(argv[0],"speed")==0){
				speed = atoi(argv[1]);
				if (speed > SPEED_MAX)	speed = SPEED_MAX;
				CCR1=5312*(SPEED_MAX+speed)/(2*SPEED_MAX);
				CCR2=5312-CCR1;
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCR1);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCR2);
				sprintf(uartTxBuffer,"Speed set to : %d | CCR1 = %d | CCR2= %d \r\n",speed,CCR1,CCR2);
				HAL_UART_Transmit(&huart2, uartTxBuffer, sizeof(uartTxBuffer), HAL_MAX_DELAY);
			}
			else{
				HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
			}
			HAL_UART_Transmit(&huart2, prompt, sizeof(prompt), HAL_MAX_DELAY);
			newCmdReady = 0;
		}

		/*
		if ((Is_First_Captured-Is_First_Captured_2)==1){

		}
		if (Is_First_Captured!=0 || Is_First_Captured_2!=0){
			if
		}
		*/

		// Start ADC Conversion
		HAL_ADC_Start(&hadc1);
		// Poll ADC1 Perihperal & TimeOut = 1mSec
		HAL_ADC_PollForConversion(&hadc1, 100);
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

/**
 * Sends the required sequence to power up the motor
 * @param[in]  None
 * @param[out] None
 */
void powerUpSequence (void){
	HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, GPIO_PIN_RESET);
}

/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		uint32_t cl = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		uint32_t ch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		frequency = (float) TIMCLOCK / (cl + 1);
	}
}
*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (Is_First_Captured==0) // if the first rising edge is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffffffff - IC_Val1) + IC_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALAR);

			frequency = refClock/Difference;

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false
		}
	}
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
			if (Is_First_Captured_2==0) // if the first rising edge is not captured
			{
				Is_First_Captured_2 = 1;  // set the first captured as true
			}

			else   // If the first rising edge is captured, now we will capture the second edge
			{
				Is_First_Captured_2 = 0; // set it back to false
			}
		}
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
