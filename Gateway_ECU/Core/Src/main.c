/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Define Warning Levels using Enumeration for better readability */
typedef enum
{
    LEVEL_SAFE,
    LEVEL_CAUTION,
    LEVEL_WARNING,
    LEVEL_DANGER,
} WarningLevel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RX_PIN 		((uint16_t)0x3000)
#define LED_TX_PIN 		((uint16_t)0xC000)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* CAN Transmission variables (To send commands to Node B) */
CAN_TxHeaderTypeDef TxHeader;
uint8_t             TxData[8];
uint32_t            TxMailbox;

/* CAN Reception variables (To receive data from Node A) */
CAN_RxHeaderTypeDef RxHeader;
uint8_t             RxData[8];

/* Logic variables */
uint16_t current_distance = 0;   /* To store the distance received from Node A */
WarningLevel_t current_level;
uint8_t previous_level = 255;

/* UART Debug (Optional if you already used printf retargeting) */
char uart_buf[100]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Logic to determine warning level based on distance
  * @param  dist: Distance in cm
  */
void Handle_Distance(uint16_t dist)
{
    /* Rule: Priority from Danger to Safe */
    if (dist > 0 && dist <= 20) {
        current_level = LEVEL_DANGER;    /* Red LED */
    }
    else if (dist > 20 && dist <= 50) {
        current_level = LEVEL_WARNING;   /* Orange LED */
    }
    else if (dist > 50 && dist <= 100) {
        current_level = LEVEL_CAUTION;   /* Blue LED */
    }
    else {
        current_level = LEVEL_SAFE;      /* Green LED */
    } 
}

///* Individual LED Control Functions */
//void Led_Safe()
//{
//	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
//}

//void Led_Caution()
//{
//	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
//}

//void Led_Warning()
//{
//	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//}

//void Led_Danger()
//{
//	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
//}

///**
//  * @brief  Refresh LED status based on the calculated current_level
//  */
//void Update_Warning_Leds(void)
//{
//    /* Reset all warning LEDs first to ensure only one is active */
//    HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);

//    /* Activate the correct LED based on current_level */
//    switch(current_level)
//    {
//        case LEVEL_SAFE:
//            Led_Safe();
//            break;
//        case LEVEL_CAUTION:
//            Led_Caution();
//            break;
//        case LEVEL_WARNING:
//            Led_Warning();
//            break;
//        case LEVEL_DANGER:
//            Led_Danger();
//            break;
//    }
//}

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
	CAN_FilterTypeDef canfilterconfig;

	/* Configuration for Filter Bank 0 to catch Sensor Data (ID: 0x250) */
	canfilterconfig.FilterBank = 0;                         /* Use Filter Bank 0 */
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;    /* Use Mask mode for flexibility */
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;   /* 32-bit scale for standard/extended ID */

	/* Target ID: 0x250 shifted left by 5 bits to align with STM32 register map */
	canfilterconfig.FilterIdHigh = 0x250 << 5;             
	canfilterconfig.FilterIdLow = 0x0000;

	/* Mask: 0x7FF means we care about all 11 bits of the Standard ID */
	canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;         
	canfilterconfig.FilterMaskIdLow = 0x0000;

	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;   /* Direct accepted messages to FIFO 0 */
	canfilterconfig.FilterActivation = ENABLE;             /* Enable this filter bank */
	canfilterconfig.SlaveStartFilterBank = 14;             /* Reserved for CAN2 (if used) */

	/* Apply filter configuration to CAN1 */
	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
	{
    /* Filter configuration Error */
    Error_Handler();
	}
	
	/* Start CAN1 peripheral */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* Enable Notifications for RX FIFO 0 message pending interrupt */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* Prepare CAN TX Header */
  TxHeader.RTR = CAN_RTR_DATA;          /* Sending actual data frame */
  TxHeader.IDE = CAN_ID_STD;            /* Using Standard 11-bit ID */
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		
		/* Process logic and update visual feedback */
    Handle_Distance(current_distance);
		
		/* We only change led state and send a new command to Node B
				if the warning level has changed.
       This technique reduces bus traffic (Anti-spamming). */
		if(current_level != previous_level)
		{
			//Update_Warning_Leds();
			/* Prepare the command message for Node B (ID: 0x450) */
      TxHeader.StdId = 0x450;
      TxHeader.DLC = 1; /* We only need 1 byte to send the level (0-3) */
      TxData[0] = current_level;
			
			/* Send the command to CAN bus */
      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
      {
				sprintf(uart_buf, "F4 Gateway -> Sending NEW Command: Level %d\r\n", current_level);
				HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	
				HAL_GPIO_TogglePin(GPIOD, LED_TX_PIN);
            
        /* Update previous state to current */
        previous_level = current_level;
      }
		}
		
    HAL_Delay(50); /* Small stability delay */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  ch: Character to be transmitted
  * @retval The character transmitted
  */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Implementation of putchar: send one character over UART */
  /* Use HAL_MAX_DELAY to ensure the character is fully transmitted */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); 
  return ch;
}

/* Put this inside HAL_CAN_RxFifo0MsgPendingCallback in Gateway code */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    /* Check if the message ID is 0x250 (Sensor Data) */
    if (RxHeader.StdId == 0x250)
		{
      /* Reconstruct the 16-bit distance from 2 bytes (Big Endian) */
      current_distance = (uint16_t)((RxData[0] << 8) | RxData[1]);

      /* Now 'received_dist' contains the distance in cm (e.g., 258) */
      /* You can now proceed to implement the Warning Logic here */
			
			HAL_GPIO_TogglePin(GPIOD, LED_RX_PIN);
            
//      /* Debug: Print the received distance to UART */
//      sprintf(uart_buf, "Gateway Received: Dist = %d cm\r\n", current_distance);
//      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
    }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
