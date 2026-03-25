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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hcsr04.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;    /* Structure for CAN TX message header */
uint8_t             TxData[2];   /* Buffer for 2 bytes of distance data */
uint32_t            TxMailbox;   /* Variable to store the assigned TX mailbox */
char                uart_buf[100]; /* Buffer for UART debug messages */

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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 1. Initialize HC-SR04 Driver */
  HCSR04_Init(&htim1);

  /* 2. Configure CAN Filter to "All-Pass" mode (Accept all incoming messages) */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterBank = 0;                         /* Use Filter Bank 0 */
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;    /* Identifier Mask mode */
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;   /* 32-bit filter scale */
  canfilterconfig.FilterIdHigh = 0x0000;                 /* Filter ID High bits */
  canfilterconfig.FilterIdLow = 0x0000;                  /* Filter ID Low bits */
  canfilterconfig.FilterMaskIdHigh = 0x0000;             /* Mask High: 0 means "Don't care" (Accept all) */
  canfilterconfig.FilterMaskIdLow = 0x0000;              /* Mask Low: 0 means "Don't care" (Accept all) */
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;   /* Assign accepted messages to FIFO 0 */
  canfilterconfig.FilterActivation = ENABLE;             /* Activate this filter */
  canfilterconfig.SlaveStartFilterBank = 14;             /* Default for single CAN peripherals */

  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* 3. Start CAN Peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
    /* CAN start Error */
		Error_Handler();
	}

  /* 4. Activate CAN Notification (Interrupt) for RX FIFO 0 */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* 5. Prepare CAN TX Header for Distance Data */
  TxHeader.StdId = 0x250;               /* Message ID for Parking Sensor Data */
  TxHeader.RTR = CAN_RTR_DATA;          /* Sending actual data frame */
  TxHeader.IDE = CAN_ID_STD;            /* Using Standard 11-bit ID */
  TxHeader.DLC = 2;                     /* Data length: 2 bytes */
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. Trigger measurement and read distance from HC-SR04 */
    float distance = HCSR04_Read();

    /* 2. Check if measurement was successful (non-negative result) */
    if (distance > 0)
    {
      /* Convert float distance (cm) to 16-bit integer */
      uint16_t dist_cm = (uint16_t)distance;

      /* 3. Pack 16-bit data into two 8-bit bytes (Big Endian) */
      TxData[0] = (uint8_t)(dist_cm >> 8);   /* High Byte (MSB) */
      TxData[1] = (uint8_t)(dist_cm & 0xFF); /* Low Byte (LSB) */

      /* 4. Request transmission of the CAN message */
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK)
      {
        /* Debug: Print transmission status to UART */
        sprintf(uart_buf, "CAN Sent: ID=0x%lX, Dist=%d cm\r\n", (unsigned long)TxHeader.StdId, dist_cm);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
				
				/* Toggle On-board LED to indicate successful transmission */
        HAL_GPIO_TogglePin(LED_NOTICE_GPIO_Port, LED_NOTICE_Pin);
            
//				if (dist_cm < 20)
//				{
//					HAL_GPIO_WritePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_TogglePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin);
//				}
//				else
//				{
//					HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
//					/* Toggle On-board LED to indicate successful transmission */
//          HAL_GPIO_TogglePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin); 
//				}
      }
    }
//    else
//    {
//      /* Debug: Notify if the sensor failed to measure (e.g., timeout) */
//      sprintf(uart_buf, "Sensor Error or Timeout\r\n");
//      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
//    }

    /* 5. Wait 100ms before the next measurement (10Hz frequency) */
    HAL_Delay(100);
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
}

/* USER CODE BEGIN 4 */
/* Common EXTI interrupt handler, this will call the driver's processing function */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SR04_Echo_Pin) /* Only call when the interrupt is from the HC-SR04 Echo pin */
    {
        HCSR04_EXTI_Callback();
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
