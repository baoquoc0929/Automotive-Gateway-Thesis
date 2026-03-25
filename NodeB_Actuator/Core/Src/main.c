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
#include "stdio.h"
#include "string.h"

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
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t         prev_tick_buzzer = 0; /* Timer for non-blocking buzzer */

/* Logic variables */
volatile uint8_t new_command_flag = 0;    /* Flag to signal new command arrived */
uint8_t          cmd_warning_level = 0;   /* 0: Safe, 1: Caution, 2: Warning, 3: Danger */
uint16_t         buzzer_interval = 0;     /* Beep interval in ms (0 = Off, 1 = Constant On) */

char uart_buf[50];

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

	/* Change 0x123 to 0x450 to match Gateway's Command ID */
	canfilterconfig.FilterIdHigh = 0x450 << 5; 
	canfilterconfig.FilterIdLow = 0x0000;

	/* Mask 0x7FF means "Check all 11 bits of the Standard ID" */
	canfilterconfig.FilterMaskIdHigh = 0x7FF << 5; 
	canfilterconfig.FilterMaskIdLow = 0x0000;

	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterActivation = ENABLE;
	canfilterconfig.SlaveStartFilterBank = 14;

	/* Apply configuration */
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

//CAN_FilterTypeDef canfilterconfig;

//canfilterconfig.FilterBank = 0;
//canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
//canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//canfilterconfig.FilterIdHigh = 0x0000;      /* Accept any ID */
//canfilterconfig.FilterIdLow = 0x0000;
//canfilterconfig.FilterMaskIdHigh = 0x0000;  /* Ignore all bits (All-Pass) */
//canfilterconfig.FilterMaskIdLow = 0x0000;
//canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//canfilterconfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* --- STEP 1: PROCESS NEW COMMAND --- */
    if (new_command_flag)
    {
        new_command_flag = 0; /* Reset flag */
        //printf("Node B -> Executing Command Level: %d\r\n", cmd_warning_level);

				/* Reset all Warning LEDs */
        HAL_GPIO_WritePin(LED_WORKING_GPIO_Port, LED_WARNING_Pin | LED_SAFETY_Pin, GPIO_PIN_RESET);

        /* Execute Actions based on Level */
        switch (cmd_warning_level)
        {
            case 0: /* SAFE */
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);  /* Servo 0 deg */
                HAL_GPIO_WritePin(LED_SAFETY_GPIO_Port, LED_SAFETY_Pin, GPIO_PIN_SET);/* LED Safety On */
                buzzer_interval = 0;                                /* Buzzer Off */
                break;

            case 1: /* CAUTION */
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000); /* Servo 45 deg */
                HAL_GPIO_WritePin(LED_SAFETY_GPIO_Port, LED_SAFETY_Pin, GPIO_PIN_SET);/* LED Warning On */
								HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
                buzzer_interval = 0;                              /* Slow beep */
                break;

            case 2: /* WARNING */
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); /* Servo 90 deg */
                HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
                buzzer_interval = 200;                              /* Fast beep */
                break;

            case 3: /* DANGER */
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2500); /* Servo 180 deg */
                HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
                buzzer_interval = 1;                                /* Constant On */
                break;
        }
    }

    /* --- STEP 2: NON-BLOCKING BUZZER CONTROL --- */
    if (buzzer_interval == 0) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); /* Force Off */
    }
    else if (buzzer_interval == 1) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   /* Force On */
    }
    else {
        /* Handle blinking (intermittent) beep levels */
        if (HAL_GetTick() - prev_tick_buzzer >= buzzer_interval) {
            HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);            /* Toggle Buzzer pin */
            prev_tick_buzzer = HAL_GetTick();                 /* Update timestamp */
        }
    }

    /* Small stabilization delay */
    HAL_Delay(10); 
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
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(LED_WORKING_GPIO_Port, LED_WORKING_Pin);

       if (RxHeader.StdId == 0x450)
        {
            new_command_flag = 1;						/* Signal main loop to process */
            cmd_warning_level = RxData[0];	/* Level 0-3 */
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
