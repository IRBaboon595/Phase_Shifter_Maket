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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
std_union				len;
std_union				temp;
l_std_union			maps_data;
l_std_union			pe_data;

uint8_t					usb_parcel_counter = 0;
uint8_t					usb_parcel_mode = 0;
uint8_t					maps_mode = PARALLEL;
uint8_t					pe_mode = PARALLEL;
uint8_t					pe_opt = OFF;

float						maps_phase = 0;
float						pe_phase = 0;

double maps_steps[6] = {180, 90, 45, 22.5};
double pe_steps[8] = {180, 90, 45, 22.5, 11.2, 5.6, 2.8, 1.4};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t USB_TX_MASS[1000];
uint8_t USB_RX_MASS[1000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
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
	//USB_TX_MASS = (uint8_t*) malloc(8);
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	len.istd = 0xFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*if(usb_parcel_counter == len.istd)
		{
			CDC_Transmit_FS(USB_RX_MASS, len.istd);
			usb_parcel_counter = 0;
		}		*/	
		//HAL_GPIO_WritePin(P_S_PE_GPIO_Port, P_S_PE_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(LE_PE_GPIO_Port, LE_PE_Pin, GPIO_PIN_SET);
		/*pe_data.listd = 0x000000E1;							//22.5
		pe_phase = pe_data.listd;
		pe_phase /= 10;
		pe_phaseshift(pe_phase, SERIAL);
		pe_data.listd = 0x000001C2;							//45
		pe_phase = pe_data.listd/10;
		pe_phaseshift(pe_phase, SERIAL);
		pe_data.listd = 0x00000384;							//90
		pe_phase = pe_data.listd/10;
		pe_phaseshift(pe_phase, SERIAL);
		pe_data.listd = 0x00000000;
		pe_phase = pe_data.listd/10;
		pe_phaseshift(pe_phase, SERIAL);*/
		/*for(int i = 0; i < usb_parcel_counter; i++)
		{
			HAL_GPIO_TogglePin(PC13_LED_GPIO_Port, PC13_LED_Pin);
			HAL_Delay(2000);
			HAL_GPIO_TogglePin(PC13_LED_GPIO_Port, PC13_LED_Pin);
			HAL_Delay(2000);
		}
		usb_parcel_counter = 0;*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 62;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC13_LED_Pin|P_S_PE_Pin|P_S_MAPS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OPT_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_LE_Pin|D2_CLK_Pin|D1_SERin_Pin|PA8_Pin
                          |PA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB0_Pin|PB1_Pin|GPIO_PIN_2|D6_SDO1_Pin
                          |D7_SDO2_Pin|LE_PE_Pin|CLK_PE_Pin|PB14_Pin
                          |SI_PE_Pin|PB3_Pin|D0_A0_Pin|D1_A1_Pin
                          |D4_LE0_Pin|D5_CLK0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13_LED_Pin P_S_PE_Pin P_S_MAPS_Pin */
  GPIO_InitStruct.Pin = PC13_LED_Pin|P_S_PE_Pin|P_S_MAPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OPT_Pin D6_Pin D5_Pin D4_Pin
                           D3_LE_Pin D2_CLK_Pin D1_SERin_Pin PA8_Pin
                           PA15_Pin */
  GPIO_InitStruct.Pin = OPT_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_LE_Pin|D2_CLK_Pin|D1_SERin_Pin|PA8_Pin
                          |PA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Sout_Pin */
  GPIO_InitStruct.Pin = Sout_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sout_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0_Pin PB1_Pin PB2 D6_SDO1_Pin
                           D7_SDO2_Pin LE_PE_Pin CLK_PE_Pin PB14_Pin
                           SI_PE_Pin PB3_Pin D0_A0_Pin D1_A1_Pin
                           D4_LE0_Pin D5_CLK0_Pin */
  GPIO_InitStruct.Pin = PB0_Pin|PB1_Pin|GPIO_PIN_2|D6_SDO1_Pin
                          |D7_SDO2_Pin|LE_PE_Pin|CLK_PE_Pin|PB14_Pin
                          |SI_PE_Pin|PB3_Pin|D0_A0_Pin|D1_A1_Pin
                          |D4_LE0_Pin|D5_CLK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void maps_control(uint16_t phase, uint8_t interface)
{
	uint8_t temp_data = 0;
	if(interface == SERIAL)
	{
		HAL_GPIO_WritePin(P_S_MAPS_GPIO_Port, P_S_MAPS_Pin, GPIO_PIN_SET);
		
		for(int i = 0; i < 4; i++);
		{
			
		}
		if(phase >= STEP_180)
		{
			phase -= STEP_45;
			temp_data += 0x20;
		}
		if(phase >= STEP_22_5)
		{
			phase -= STEP_22_5;
			temp_data += 0x80;
		}
		if(phase >= STEP_45)
		{
			phase -= STEP_45;
			temp_data += 0x20;
		}
		if(phase >= STEP_90)
		{
			phase -= STEP_45;
			temp_data += 0x20;
		}
		if(phase >= STEP_180)
		{
			phase -= STEP_45;
			temp_data += 0x20;
		}
		if(phase >= STEP_45)
		{
			phase -= STEP_45;
			temp_data += 0x20;
		}			
		
	}
	else if(interface == PARALLEL)
	{
		HAL_GPIO_WritePin(P_S_MAPS_GPIO_Port, P_S_MAPS_Pin, GPIO_PIN_RESET);
	}
}
	
void usb_irq_parser(void)
{
	std_union usb_len;
	uint8_t temp = 0;
	usb_len.cstd[1] = USB_RX_MASS[1];
	usb_len.cstd[0] = USB_RX_MASS[2];
	HAL_GPIO_TogglePin(PC13_LED_GPIO_Port, PC13_LED_Pin);
	switch(USB_RX_MASS[4])
	{
		case	ECHO:
			CDC_Transmit_FS(USB_RX_MASS, usb_len.istd);
		break;
		case	MAPS_P_S:
			maps_mode = USB_RX_MASS[5];
			if(maps_mode)
			{
				HAL_GPIO_WritePin(P_S_MAPS_GPIO_Port, P_S_MAPS_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(P_S_MAPS_GPIO_Port, P_S_MAPS_Pin, GPIO_PIN_RESET);
			}
		break;
		case 	MAPS_DATA:
			maps_data.cstd[3] = USB_RX_MASS[5];
			maps_data.cstd[2] = USB_RX_MASS[6];
			maps_data.cstd[1] = USB_RX_MASS[7];
			maps_data.cstd[0] = USB_RX_MASS[8];
			maps_phase = maps_data.listd/10;
			maps_phaseshift(maps_phase, maps_mode);
		break;
		case	PE_P_S:
			pe_mode = USB_RX_MASS[5];
			if(pe_mode)
			{
				HAL_GPIO_WritePin(P_S_PE_GPIO_Port, P_S_PE_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(P_S_PE_GPIO_Port, P_S_PE_Pin, GPIO_PIN_RESET);
			}
		break;
		case 	PE_DATA:
			pe_data.cstd[3] = USB_RX_MASS[5];
			pe_data.cstd[2] = USB_RX_MASS[6];
			pe_data.cstd[1] = USB_RX_MASS[7];
			pe_data.cstd[0] = USB_RX_MASS[8];
			pe_phase = pe_data.listd/10;
			pe_phaseshift(pe_phase, pe_mode);
		break;
		case	PE_OPT:
			pe_opt = USB_RX_MASS[5];
			if(pe_opt)
			{
				HAL_GPIO_WritePin(OPT_GPIO_Port, OPT_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(OPT_GPIO_Port, OPT_Pin, GPIO_PIN_RESET);
			}
		break;
			case I2C_REQUEST:
				USB_RX_MASS[0] = 0x11;
				memcpy(USB_TX_MASS, "i2c message", 11);
				CDC_Transmit_FS(USB_TX_MASS, 11);
				//temp = 0x11;
				HAL_I2C_Master_Transmit(&hi2c1, 0x3E, USB_RX_MASS, 1, 100);
			break;
		default:
			memcpy(USB_TX_MASS, "No message", 10);
			CDC_Transmit_FS(USB_RX_MASS, usb_len.istd);
		break;
	}
	usb_parcel_counter = 0;
}

uint8_t xor_handler(uint8_t *mass)
{
	uint8_t result = 0;
	std_union temp_1;
	temp_1.cstd[1] = mass[1];
	temp_1.cstd[0] = mass[2];
	for(int i = 0; i < (temp_1.istd); i++)
	{
		result ^= mass[i];
	}
	return result;
}

void maps_phaseshift(float phase, uint8_t mode)
{
	std_union maps_step;			
	float phase_temp = phase;
	maps_step.istd = 0;
	if(mode == PARALLEL)
	{
		maps_step.istd = GPIOA->IDR;
		for(int i = 0; i < 4; i++)
		{
			phase -= maps_steps[i];
			if(phase >= 0)
			{
				maps_step.istd |= (1 << (1 + i));																						//Fill the phase byte so GPIOA pins 1, 2, 3, 4 are used
				phase_temp = phase;
			}
			else
			{
				maps_step.istd &=~ (1 << (1 + i));
				phase = phase_temp;
			}
		}
		maps_step.istd += 160;																			//Dummy bits tight to logic 1
		GPIOA->ODR = maps_step.istd;
	}
	else if(mode == SERIAL)
	{
		for(int i = 0; i < 4; i++)
		{
			phase -= maps_steps[i];
			if(phase >= 0)
			{
				maps_step.cstd[0] |= (1 << (5 - i));
				phase_temp = phase;
			}
			else
			{
				maps_step.cstd[0] &=~ (1 << (5 - i));
				phase = phase_temp;
			}
		}	
		maps_step.cstd[0] |= 3; 																														//Two LSBs in SPI mode are not used
		HAL_GPIO_WritePin(D3_LE_GPIO_Port, D3_LE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D2_CLK_GPIO_Port, D2_CLK_Pin, GPIO_PIN_RESET);
		
		for(int i = 6; i >= 1; i--)
		{
			if((maps_step.cstd[0] & (1 << (i - 1))) != 0)
			{
				HAL_GPIO_WritePin(D1_SERin_GPIO_Port, D1_SERin_Pin, GPIO_PIN_SET);
			}
			else if((maps_step.cstd[0] & (1 << (i - 1))) == 0)
			{
				HAL_GPIO_WritePin(D1_SERin_GPIO_Port, D1_SERin_Pin, GPIO_PIN_RESET);
			}
			
			HAL_GPIO_WritePin(D2_CLK_GPIO_Port, D2_CLK_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D2_CLK_GPIO_Port, D2_CLK_Pin, GPIO_PIN_RESET);
		}	
		
		HAL_GPIO_WritePin(D3_LE_GPIO_Port, D3_LE_Pin, GPIO_PIN_SET);
		
	}
}

void pe_phaseshift(float phase, uint8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	std_union pe_step;
	pe_step.istd = 0;
	float phase_temp = phase;
	if(mode == PARALLEL)
	{
		HAL_GPIO_WritePin(LE_PE_GPIO_Port, LE_PE_Pin, GPIO_PIN_SET);
		GPIO_InitStruct.Pin = D6_SDO1_Pin|D7_SDO2_Pin|D4_LE0_Pin|D5_CLK0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		pe_step.istd = GPIOB->IDR;
		for(int i = 0; i < 8; i++)
		{
			phase -= pe_steps[i];
			if(phase >= 0)
			{
				pe_step.istd |= (1 << (11 - i));					//Fill the phase byte so GPIOB pins 4 to 11 are used
				phase_temp = phase;
			}
			else
			{
				pe_step.istd &=~ (1 << (11 - i));
				phase = phase_temp;
			}
		}
		//pe_step &= 0x1E; 
		GPIOB->ODR = pe_step.istd;
		HAL_GPIO_WritePin(LE_PE_GPIO_Port, LE_PE_Pin, GPIO_PIN_RESET);
	}
	else if(mode == SERIAL)
	{
		GPIO_InitStruct.Pin = D6_SDO1_Pin|D7_SDO2_Pin|D4_LE0_Pin|D5_CLK0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		pe_step.istd = GPIOB->IDR;
		HAL_GPIO_WritePin(D0_A0_GPIO_Port,D0_A0_Pin, GPIO_PIN_RESET);						//Setting Address - 0x00
		HAL_GPIO_WritePin(D1_A1_GPIO_Port,D1_A1_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(D2_A2_GPIO_Port,D2_A2_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(D3_A3_GPIO_Port,D3_A3_Pin, GPIO_PIN_RESET);
		for(int i = 0; i < 8; i++)
		{
			phase -= pe_steps[i];
			if(phase >= 0)
			{
				pe_step.istd |= (1 << (7 - i));
				phase_temp = phase;
			}
			else
			{
				pe_step.istd &=~ (1 << (7 - i));
				phase = phase_temp;
			}
		}	
		HAL_GPIO_WritePin(LE_PE_GPIO_Port, LE_PE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLK_PE_GPIO_Port, CLK_PE_Pin, GPIO_PIN_RESET);
		
		for(int i = 0; i < 8; i++)
		{
			if((pe_step.cstd[0] & (1 << i)) != 0)
			{
				HAL_GPIO_WritePin(SI_PE_GPIO_Port, SI_PE_Pin, GPIO_PIN_SET);
			}
			else if((pe_step.cstd[0] & (1 << i)) == 0)
			{
				HAL_GPIO_WritePin(SI_PE_GPIO_Port, SI_PE_Pin, GPIO_PIN_RESET);
			}
			
			HAL_GPIO_WritePin(CLK_PE_GPIO_Port, CLK_PE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CLK_PE_GPIO_Port, CLK_PE_Pin, GPIO_PIN_RESET);
		}	
		HAL_GPIO_WritePin(SI_PE_GPIO_Port, SI_PE_Pin, GPIO_PIN_RESET);
		for(int i = 0; i < 5; i++)
		{
			HAL_GPIO_WritePin(CLK_PE_GPIO_Port, CLK_PE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CLK_PE_GPIO_Port, CLK_PE_Pin, GPIO_PIN_RESET);
		}
		
		HAL_GPIO_WritePin(LE_PE_GPIO_Port, LE_PE_Pin, GPIO_PIN_SET);
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
