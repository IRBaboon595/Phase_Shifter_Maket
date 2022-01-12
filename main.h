/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union _std_union{
	uint16_t 	istd;
	uint8_t 	cstd[2];
}std_union;

typedef union _l_std_union{
	uint32_t	listd;
	uint16_t 	istd[2];
	uint8_t 	cstd[4];
}l_std_union;

typedef union _l_l_std_union{
	uint64_t	llistd;
	uint32_t	listd[2];
	uint16_t 	istd[4];
	uint8_t 	cstd[8];
}l_l_std_union;

extern uint8_t 						USB_TX_MASS[1000];
extern uint8_t 						USB_RX_MASS[1000];

extern uint8_t						usb_parcel_counter;
extern uint8_t						usb_parcel_mode;

extern std_union					len;
extern std_union					temp;
extern l_std_union				maps_data;
extern l_std_union				pe_data;
extern uint8_t						pe_opt;
extern uint8_t						maps_mode;
extern uint8_t						pe_mode;
extern uint8_t						pe_opt;

extern float							maps_phase;
extern float							pe_phase;

extern double 						maps_steps[6];
extern double 						pe_steps[8];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void usb_irq_parser(void);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern uint8_t xor_handler(uint8_t *mass);
void maps_phaseshift(float phase, uint8_t mode);
void pe_phaseshift(float phase, uint8_t mode);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_LED_Pin GPIO_PIN_13
#define PC13_LED_GPIO_Port GPIOC
#define P_S_PE_Pin GPIO_PIN_14
#define P_S_PE_GPIO_Port GPIOC
#define P_S_MAPS_Pin GPIO_PIN_15
#define P_S_MAPS_GPIO_Port GPIOC
#define OPT_Pin GPIO_PIN_0
#define OPT_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_1
#define D6_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_2
#define D5_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_3
#define D4_GPIO_Port GPIOA
#define D3_LE_Pin GPIO_PIN_4
#define D3_LE_GPIO_Port GPIOA
#define D2_CLK_Pin GPIO_PIN_5
#define D2_CLK_GPIO_Port GPIOA
#define Sout_Pin GPIO_PIN_6
#define Sout_GPIO_Port GPIOA
#define D1_SERin_Pin GPIO_PIN_7
#define D1_SERin_GPIO_Port GPIOA
#define PB0_Pin GPIO_PIN_0
#define PB0_GPIO_Port GPIOB
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define D6_SDO1_Pin GPIO_PIN_10
#define D6_SDO1_GPIO_Port GPIOB
#define D7_SDO2_Pin GPIO_PIN_11
#define D7_SDO2_GPIO_Port GPIOB
#define LE_PE_Pin GPIO_PIN_12
#define LE_PE_GPIO_Port GPIOB
#define CLK_PE_Pin GPIO_PIN_13
#define CLK_PE_GPIO_Port GPIOB
#define PB14_Pin GPIO_PIN_14
#define PB14_GPIO_Port GPIOB
#define SI_PE_Pin GPIO_PIN_15
#define SI_PE_GPIO_Port GPIOB
#define PA8_Pin GPIO_PIN_8
#define PA8_GPIO_Port GPIOA
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define PB3_Pin GPIO_PIN_3
#define PB3_GPIO_Port GPIOB
#define D0_A0_Pin GPIO_PIN_4
#define D0_A0_GPIO_Port GPIOB
#define D1_A1_Pin GPIO_PIN_5
#define D1_A1_GPIO_Port GPIOB
#define D4_LE0_Pin GPIO_PIN_8
#define D4_LE0_GPIO_Port GPIOB
#define D5_CLK0_Pin GPIO_PIN_9
#define D5_CLK0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define OFF											0
#define ON											1

#define PARALLEL 								0
#define SERIAL									1

/*************************** MAPS PHASE STEPS ***************************/

#define STEP_22_5								225
#define STEP_45									450
#define STEP_90									900
#define STEP_180								1800
#define STEP_337_5							3375

/*************************** PE44820 PHASE STEPS ***************************/

#define STEP_1_4								14
#define STEP_2_8								28
#define STEP_5_6								56
#define STEP_11_2								112
//#define STEP_22_5							225
//#define STEP_45								450
//#define STEP_90								900
//#define STEP_180							1800
#define STEP_358_6							3586

#define PE_ADDRESS							0x00
	
/*************************** USB PROTOCOL **********************************/

//SERVICE BYTES
#define SYNCHRO									0x02
#define	DEV_ADDRESS							0x0A

//FUNCTIONS
#define ECHO										0x00
#define MAPS_P_S								0x01
#define	MAPS_DATA								0x02
#define PE_P_S									0x03
#define	PE_DATA									0x04
#define PE_OPT									0x05
#define I2C_REQUEST							0x06



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
