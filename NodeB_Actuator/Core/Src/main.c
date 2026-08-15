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
#define UDS_SERVICE_ECU_RESET    (0x11) /*!< UDS Service ID for ECU Reset */
#define UDS_SERVICE_IO_CONTROL   (0x2F) /*!< UDS Service ID for Input/Output Control */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;             /**< CAN Rx header structure */
uint8_t             RxData[8];            /**< CAN Rx payload data array */

volatile uint8_t    uds_rx_flag = 0;      /**< Flag indicating a new UDS request was received */
uint8_t             uds_rx_data[8];       /**< UDS Rx payload data array */
uint8_t             uds_dlc = 0;          /**< UDS Data Length Code (Payload size) */

volatile uint8_t    new_command_flag = 0; /**< Flag indicating a new manual command arrived */
uint8_t             cmd_warning_level = 0;/**< Warning state -> 0: Safe, 1: Caution, 2: Warning, 3: Danger */
uint16_t            buzzer_interval = 0;  /**< Buzzer toggle interval in ms (0 = Off, 1 = Constant On) */
uint32_t            prev_tick_buzzer = 0; /**< Timer tracker for non-blocking buzzer execution */

char                uart_buf[128];        /**< Buffer for UART transmission */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param[in] ch Character to be transmitted
 * @return Transmitted character
 */
PUTCHAR_PROTOTYPE;

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
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;   
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;  

  /* Slot 1: Accept Control Command (0x450) */
  canfilterconfig.FilterIdHigh = 0x450 << 5; 
  /* Slot 2: Accept UDS Request (0x7E0) */
  canfilterconfig.FilterIdLow = 0x7E0 << 5;  
  
  /* Slot 3 & 4: Duplicate to safely fill all slots */
  canfilterconfig.FilterMaskIdHigh = 0x450 << 5; 
  canfilterconfig.FilterMaskIdLow = 0x7E0 << 5;  

  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterActivation = ENABLE;
  canfilterconfig.SlaveStartFilterBank = 14;

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
  
  /* Diagnostic override timer for UDS 0x2F command */
  uint32_t buzzer_test_timeout = 0; 

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* --- STEP 1: PROCESS APPLICATION COMMANDS --- */
//    if (new_command_flag)
//    {
//      new_command_flag = 0; 
//      HAL_GPIO_WritePin(LED_WORKING_GPIO_Port, LED_CAUTION_Pin | LED_SAFE_Pin, GPIO_PIN_RESET);

//      switch (cmd_warning_level)
//      {
//        case 0: /* SAFE */
//          HAL_GPIO_WritePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin, GPIO_PIN_SET);
//          buzzer_interval = 0;                                
//          break;
//        case 1: /* CAUTION */
//          HAL_GPIO_WritePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin, GPIO_PIN_SET);
//          HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin, GPIO_PIN_SET);
//          buzzer_interval = 1000;                              
//          break;
//        case 2: /* WARNING */
//          HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin, GPIO_PIN_SET);
//          buzzer_interval = 500;                              
//          break;
//        case 3: /* DANGER */
//          HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin, GPIO_PIN_SET);
//          buzzer_interval = 200;                                
//          break;
//      }
//    }
		
		if (new_command_flag)
    {
      new_command_flag = 0; 
      HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin | LED_SAFE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_DANGER_GPIO_Port, LED_DANGER_Pin, GPIO_PIN_RESET);

      switch (cmd_warning_level)
      {
        case 0: /* SAFE */
          HAL_GPIO_WritePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin, GPIO_PIN_SET);
          buzzer_interval = 0;                                
          break;
        case 1: /* CAUTION */
					HAL_GPIO_WritePin(LED_SAFE_GPIO_Port, LED_SAFE_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin, GPIO_PIN_SET);
          buzzer_interval = 1000;                              
          break;
        case 2: /* WARNING */
          HAL_GPIO_WritePin(LED_CAUTION_GPIO_Port, LED_CAUTION_Pin, GPIO_PIN_SET);
          buzzer_interval = 500;                              
          break;
        case 3: /* DANGER */
          HAL_GPIO_WritePin(LED_DANGER_GPIO_Port, LED_DANGER_Pin, GPIO_PIN_SET);
          buzzer_interval = 200;                                
          break;
      }
    }

    /* --- STEP 2: ACTUATOR CONTROL LAYER (OVERRIDE) --- */
    if (buzzer_test_timeout > 0 && (HAL_GetTick() - buzzer_test_timeout < 2000))
    {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); 
    }
    else
    {
      if (buzzer_test_timeout > 0) buzzer_test_timeout = 0;

      if (buzzer_interval == 0)      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); 
      else if (buzzer_interval == 1) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   
      else
      {
        if (HAL_GetTick() - prev_tick_buzzer >= buzzer_interval)
        {
          HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);              
          prev_tick_buzzer = HAL_GetTick();                              
        }
      }
    }

    /* --- STEP 3: UDS DIAGNOSTIC SERVICE ROUTER --- */
    if (uds_rx_flag == 1)
    {
      uds_rx_flag = 0;
      
      /* Service 0x11: ECU Reset */
      if (uds_rx_data[1] == UDS_SERVICE_ECU_RESET && uds_rx_data[2] == 0x01)
      {
          printf("\r\n[NODE B] Nhan lenh ECU RESET tu Gateway!\r\n");

          CAN_TxHeaderTypeDef UdsTxHeader = {0x7E8, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
          uint8_t UdsTxData[8] = {0x02, 0x51, 0x01, 0x55, 0x55, 0x55, 0x55, 0x55};
          uint32_t UdsTxMailbox;

          if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
              HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
          }
          HAL_CAN_AddTxMessage(&hcan, &UdsTxHeader, UdsTxData, &UdsTxMailbox);

          printf("[NODE B] Da gui Response. Chuan bi Reset chip...\r\n");

          HAL_Delay(100);     
          NVIC_SystemReset();  
      }
      /* Service 0x2F: IO Control (Test Buzzer) */
      else if (uds_rx_data[1] == UDS_SERVICE_IO_CONTROL && uds_rx_data[2] == 0x01)
      {
          printf("\r\n[NODE B] Nhan lenh IO CONTROL (0x2F)! Ep coi keu 2 giay.\r\n");

          buzzer_test_timeout = HAL_GetTick();
          if(buzzer_test_timeout == 0) buzzer_test_timeout = 1; 

          CAN_TxHeaderTypeDef UdsTxHeader = {0x7E8, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
          uint8_t UdsTxData[8] = {0x02, 0x6F, 0x01, 0x55, 0x55, 0x55, 0x55, 0x55};
          uint32_t UdsTxMailbox;

          if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
              HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
          }
          HAL_CAN_AddTxMessage(&hcan, &UdsTxHeader, UdsTxData, &UdsTxMailbox);
          
          printf("[NODE B] Da gui Response UDS IO Control.\r\n");
      }
    }

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

/* Retargets the C library printf function to the USART */
PUTCHAR_PROTOTYPE
{
  /* Timeout lowered to 10ms to prevent system hang */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10); 
  return ch;
}

/* CAN RX interrupt callback (Executed when message arrives) */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      HAL_GPIO_TogglePin(LED_WORKING_GPIO_Port, LED_WORKING_Pin);

      /* Branch 1: Application Layer (Normal Control) */
      if (RxHeader.StdId == 0x450)
      {
        new_command_flag = 1;          
        cmd_warning_level = RxData[0]; 
      }
      /* Branch 2: Diagnostic Layer (UDS Requests) */
      else if (RxHeader.StdId == 0x7E0)
      {
        uds_rx_flag = 1;               
        uds_dlc = RxHeader.DLC;        
        
        /* Safely copy payload without <string.h> */
        for(uint8_t i = 0; i < RxHeader.DLC; i++)
        {
          uds_rx_data[i] = RxData[i];
        }
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
