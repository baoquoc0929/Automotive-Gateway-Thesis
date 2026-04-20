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
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "lwip/udp.h"  /* Required for UDP functions */
#include <string.h>    /* Required for strlen and sprintf */

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

/* Telemetry & Logic variables */
uint16_t current_distance = 0;   /* Distance received from Node A (cm) */
WarningLevel_t current_level;    /* Calculated warning level (0-3) */
uint8_t previous_level = 255;    /* Memory to detect level changes */

/* Operating Modes */
uint8_t control_mode = 0;        /* 0: AUTO (Sensor-based), 1: MANUAL (PC-based) */
uint32_t last_manual_time = 0;   /* Timestamp of last received PC command */
const uint32_t MANUAL_TIMEOUT = 5000; /* 5 seconds safety timeout */

/* Flags for optimized processing (Non-blocking) */
volatile uint8_t new_can_data_flag = 0; /* Set in CAN IRQ, processed in main loop */

/* UART Debug Buffer */
char uart_buf[100]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes */
void Gateway_UDP_Receiver_Init(void);  /* Add this prototype here */
void Gateway_Send_UDP(uint16_t dist_cm); /* Should add this too for safety */

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

///**
//  * @brief  Turn on the corresponding LED on the F4 Discovery board 
//  *         based on the current safety level.
//  *         LD4 (Green)  : Safe
//  *         LD6 (Blue)   : Caution
//  *         LD3 (Orange) : Warning
//  *         LD5 (Red)    : Danger
//  */
//void Update_Warning_Leds(void)
//{
//    /* 1. First, turn OFF all 4 LEDs to clear the previous state */
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

//    /* 2. Turn ON only the LED that matches the current level */
//    /* Note: current_level or current_warning_level is the variable you used to store 0-3 */
//    switch (current_level) 
//    {
//        case 0: /* SAFE */
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); /* Green LED (LD4) */
//            break;
//        case 1: /* CAUTION */
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); /* Blue LED (LD6) */
//            break;
//        case 2: /* WARNING */
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); /* Orange LED (LD3) */
//            break;
//        case 3: /* DANGER */
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); /* Red LED (LD5) */
//            break;
//        default:
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
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
	/* Initialize the UDP Listener */
	Gateway_UDP_Receiver_Init();
	
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
	TxHeader.StdId = 0x450;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 1;
  TxHeader.TransmitGlobalTime = DISABLE;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* REQUIRED: Handle LwIP background tasks and packet processing */
		MX_LWIP_Process();
		
		/* --- TELEMETRY: Send distance data to PC via UDP --- */
    /* We process this in the main loop to keep Interrupt duration minimal */
    if (new_can_data_flag == 1)
    {
      Gateway_Send_UDP(current_distance);
      new_can_data_flag = 0; /* Reset flag after transmission */
    }
		
		/* --- STEP 1: SAFETY CHECK - Manual Mode Timeout --- */
    if (control_mode == 1) 
    {
      /* Return to AUTO mode if no command received within timeout period */
      if (HAL_GetTick() - last_manual_time > MANUAL_TIMEOUT) 
      {
        control_mode = 0; 
        previous_level = 255; /* Force an update in the next cycle */
      }
    }
		
    /* --- STEP 2: CONTROL LOGIC - Only runs in AUTO mode --- */
    if (control_mode == 0) 
    {
      Handle_Distance(current_distance);

      /* Only send CAN message if the warning level has changed (Event-driven) */
      if (current_level != previous_level)
      {
				TxData[0] = current_level;
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
        {
           previous_level = current_level; /* Successfully updated */
        }
      }
    }
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
///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  ch: Character to be transmitted
//  * @retval The character transmitted
//  */
//#ifdef __GNUC__
///* With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */

//PUTCHAR_PROTOTYPE
//{
//  /* Implementation of putchar: send one character over UART */
//  /* Use HAL_MAX_DELAY to ensure the character is fully transmitted */
//  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY); 
//  return ch;
//}

/**
  * @brief  Constructs and sends a UDP packet containing distance data to PC.
  * @param  dist_cm: Sensor value to be transmitted.
  */
void Gateway_Send_UDP(uint16_t dist_cm)
{
  struct udp_pcb *upcb;
  struct pbuf *p;
  ip_addr_t DestIPaddr;
  char msg_buffer[50];

	/* Create a new UDP control block */
  upcb = udp_new();
	
  if (upcb != NULL)
  {
		/* Destination PC IP Address */
    IP4_ADDR(&DestIPaddr, 192, 168, 1, 100); 

    /* Format string for Python App parsing: "Dist: X cm" */
    sprintf(msg_buffer, "Dist: %d cm\r\n", dist_cm);

    p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg_buffer), PBUF_RAM);	
    if (p != NULL)
    {
      pbuf_take(p, msg_buffer, strlen(msg_buffer));
      udp_sendto(upcb, p, &DestIPaddr, 8080);
      pbuf_free(p);
    }
    udp_remove(upcb); /* Release UDP control block */
  }
}

/**
  * @brief  CAN RX Callback - Called when a new message arrives in FIFO0.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
		/* Filter for Sensor Data from Node A (ID: 0x250) */
    if (RxHeader.StdId == 0x250)
    {
			/* Reconstruct the 16-bit distance from 2 bytes (Big Endian) */
      current_distance = (uint16_t)((RxData[0] << 8) | RxData[1]);
			
			/* Raise flag for main loop to handle UDP transmission */
      new_can_data_flag = 1;
    }
  }
}

/**
  * @brief  Callback function called by LwIP when a UDP packet is received
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  if (p != NULL)
  {
    uint8_t received_char = *((uint8_t *)p->payload);
        
    /* Command '0'-'3': Activate MANUAL Mode and forward to Node B */
    if (received_char >= '0' && received_char <= '3') 
    {
      control_mode = 1; 
      last_manual_time = HAL_GetTick(); /* Reset timeout timer */
            
      TxHeader.StdId = 0x450;
      TxHeader.DLC = 1;
      TxData[0] = received_char - '0';
      HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
    /* Command 'A': Manually restore AUTO Mode */
    else if (received_char == 'A' || received_char == 'a')
    {
      control_mode = 0;
      previous_level = 255; /* Force immediate update */
    }
        
    pbuf_free(p); /* MANDATORY: Release buffer after processing */
  }
}

/**
  * @brief  Initialize UDP Listener for incoming PC commands.
  */
void Gateway_UDP_Receiver_Init(void)
{
    struct udp_pcb *upcb = udp_new();
    if (upcb != NULL)
    {
        if (udp_bind(upcb, IP_ADDR_ANY, 8080) == ERR_OK)
        {
            udp_recv(upcb, udp_receive_callback, NULL);
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
