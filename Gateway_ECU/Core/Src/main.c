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
#include "cmsis_os.h"
#include "can.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>     /* Required for printf and sprintf */
#include <string.h>    /* Required for strlen */
#include "lwip/udp.h"  /* Required for UDP functions */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Network Configuration */
#define PC_DEST_IP_0            (192)
#define PC_DEST_IP_1            (168)
#define PC_DEST_IP_2            (1)
#define PC_DEST_IP_3            (100)
#define GATEWAY_UDP_PORT        (8080)

/* CAN Protocol IDs */
#define CAN_ID_NODE_A_RX        (0x250)
#define CAN_ID_GATEWAY_APP_TX   (0x450)
#define CAN_ID_GATEWAY_UDS_TX   (0x7E0)
#define CAN_ID_NODE_B_UDS_RX    (0x7E8)

/* Event Flags */
#define EVT_UDS_RX_BIT   0x00000001U
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
/* CAN Communication Buffers */
CAN_TxHeaderTypeDef TxHeader;               /**< CAN Tx header structure (To Node B) */
uint8_t             TxData[8];              /**< CAN Tx payload data array */
uint32_t            TxMailbox;              /**< CAN Tx mailbox identifier */

CAN_RxHeaderTypeDef RxHeader;               /**< CAN Rx header structure (From Node A & B) */
uint8_t             RxData[8];              /**< CAN Rx payload data array */

/* UDS Diagnostic Protocol Variables */
uint8_t             gateway_uds_level = 0;  /**< Extracted UDS response payload */

/* Telemetry & System Logic Variables */
uint16_t            current_distance = 0;   /**< Distance received from Node A (cm) */
WarningLevel_t      current_level;          /**< Calculated warning level (0:Safe -> 3:Danger) */
uint8_t             previous_level = 255;   /**< Memory to detect level changes (255 = init) */

uint8_t             control_mode = 0;       /**< 0: AUTO (Sensor), 1: MANUAL (PC-based) */
uint32_t            last_manual_time = 0;   /**< Timestamp of last received PC command */
const uint32_t      MANUAL_TIMEOUT = 5000;  /**< 5 seconds safety timeout for manual mode */

/* UART Debugging Buffer */
char                uart_buf[250];          /**< Buffer for UART transmission */

/* External Variables */
extern osMessageQueueId_t CAN_Data_QueueHandle;

extern osEventFlagsId_t UdsEventFlagsHandle;

volatile uint32_t can_sensor_start_time;    // Timestamp for CAN message from Node A (Sensor)
volatile uint32_t can_uds_start_time;       // Timestamp for UDS response from Node B

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param[in] ch Character to be transmitted
 * @return Transmitted character
 */
PUTCHAR_PROTOTYPE;

/* Private function prototypes */
/**
 * @brief  Initializes the UDP receiver for Gateway.
 */
void Gateway_UDP_Receiver_Init(void);

/**
 * @brief  Sends distance telemetry over UDP to the PC.
 * @param[in] dist_cm Distance value in centimeters.
 */
void Gateway_Send_UDP(uint16_t dist_cm);

/**
 * @brief  Formats and sends the UDS acknowledgment level via UDP to the PC.
 * @param[in] level The UDS response level received from Node B.
 */
void Gateway_Send_UDS_UDP(uint8_t level);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t filtered_distance = 0; /**< Filtered distance state for moving average */

/**
 * @brief  Determine warning level using a smoothed distance value.
 * @param[in] dist Raw distance in cm from sensor.
 */
void Handle_Distance(uint16_t dist)
{
  /* Moving average filter: Retain 70% of previous value, blend with 30% new value */
  if(filtered_distance == 0)
  {
    filtered_distance = dist; 
  }
  
  filtered_distance = (filtered_distance * 7 + dist * 3) / 10;

  /* Evaluate safety level using the filtered data */
  if(filtered_distance > 0 && filtered_distance <= 20)
  {
    current_level = LEVEL_DANGER;
  }
  else if(filtered_distance > 20 && filtered_distance <= 50) 
  {
    current_level = LEVEL_WARNING;
  }
  else if(filtered_distance > 50 && filtered_distance <= 100)
  {
    current_level = LEVEL_CAUTION;
  }
  else
  {
    current_level = LEVEL_SAFE;
  }
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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
/* USER CODE BEGIN 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;   
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;

  /* Accept Node A (0x250) and Node B UDS Response (0x7E8) */
  canfilterconfig.FilterIdHigh = 0x250 << 5;            
  canfilterconfig.FilterIdLow = 0x7E8 << 5;             

  /* Duplicate IDs to safely fill all 16-bit register slots */
  canfilterconfig.FilterMaskIdHigh = 0x250 << 5; 
  canfilterconfig.FilterMaskIdLow = 0x7E8 << 5;  

  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterActivation = ENABLE;
  canfilterconfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare default CAN TX Header */
  TxHeader.StdId = 0x450;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 1;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
 *
 * @param[in]  ch  Character to be transmitted
 *
 * @return 
 * - Transmitted character
 */
PUTCHAR_PROTOTYPE
{
  /* Timeout lowered to 10ms to prevent system hang */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 10); 
  return ch;
}

/**
 * @brief  Helper function to send a raw string via UDP.
 *
 * @param[in]  msg_string  Pointer to the null-terminated string to send.
 *
 * @return None
 */
void UDP_Send_String(const char *msg_string)
{
  struct udp_pcb *upcb = udp_new();
  if (upcb != NULL)
  {
    ip_addr_t DestIPaddr;
    IP4_ADDR(&DestIPaddr, PC_DEST_IP_0, PC_DEST_IP_1, PC_DEST_IP_2, PC_DEST_IP_3); 

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg_string), PBUF_RAM);  
    if (p != NULL)
    {
      pbuf_take(p, msg_string, strlen(msg_string));
      udp_sendto(upcb, p, &DestIPaddr, GATEWAY_UDP_PORT);
      pbuf_free(p);
    }
    udp_remove(upcb); 
  }
}

/**
 * @brief  Formats and sends the distance telemetry via UDP to the PC.
 *
 * @param[in]  dist_cm  Distance value in centimeters.
 *
 * @return None
 */
void Gateway_Send_UDP(uint16_t dist_cm)
{
  char msg_buffer[50];
  sprintf(msg_buffer, "Dist: %d cm\r\n", dist_cm);
  UDP_Send_String(msg_buffer);
}

/**
 * @brief  Formats and sends the UDS acknowledgment level via UDP to the PC.
 *
 * @param[in]  level  The UDS response level received from Node B.
 *
 * @return None
 */
void Gateway_Send_UDS_UDP(uint8_t level)
{
  char msg_buffer[50];
  sprintf(msg_buffer, "UDS_ACK:%d\r\n", level); 
  UDP_Send_String(msg_buffer);
}

/**
 * @brief  CAN RX Callback - Triggered when a new message arrives in FIFO0.
 *
 * @param[in]  hcan  Pointer to a CAN_HandleTypeDef structure that contains
 * the configuration information for the specified CAN.
 *
 * @return None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    /* Branch 1: Sensor Data from Node A */
    if (RxHeader.StdId == CAN_ID_NODE_A_RX)
    {
      /* Capture timestamp for latency measurement */
      can_sensor_start_time = DWT_GET();

      /* Extract distance value from CAN payload (2 bytes) */
      uint16_t dist_val = (uint16_t)((RxData[0] << 8) | RxData[1]);
			
      /* Push raw distance into the queue. Timeout is 0 because this is inside ISR */
      osMessageQueuePut(CAN_Data_QueueHandle, &dist_val, 0, 0);
    }
    /* Branch 2: UDS Response from Node B */
    else if (RxHeader.StdId == CAN_ID_NODE_B_UDS_RX)
    {
      /* Capture timestamp for latency measurement */
      can_uds_start_time = DWT_GET();

      /* Extract UDS response level from CAN payload (1 byte) */
      osEventFlagsSet(UdsEventFlagsHandle, EVT_UDS_RX_BIT);
      gateway_uds_level = RxData[1]; 
    }
  }
}

/**
 * @brief  LwIP UDP RX Callback - Triggered when a PC command arrives via Ethernet.
 *
 * @param[in]  arg   User supplied argument (not used).
 * @param[in]  upcb  The UDP protocol control block.
 * @param[in]  p     The packet buffer containing the received data.
 * @param[in]  addr  The IP address of the sender.
 * @param[in]  port  The port number of the sender.
 *
 * @attention  Must call pbuf_free(p) at the end to prevent memory leaks in LwIP.
 *
 * @return None
 */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  if (p != NULL)
  {
    uint8_t *payload = (uint8_t *)p->payload;
    uint8_t received_char = payload[0];
        
    /* Branch 1: MANUAL Mode Control ('0'-'3') */
    if (received_char >= '0' && received_char <= '3') 
    {
      control_mode = 1; 
      last_manual_time = HAL_GetTick(); 
            
      TxHeader.StdId = CAN_ID_GATEWAY_APP_TX;
      TxHeader.DLC = 1;
      TxData[0] = received_char - '0';
      
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
          HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
      }
      HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
    /* Branch 2: Restore AUTO Mode ('A' or 'a') */
    else if (received_char == 'A' || received_char == 'a')
    {
      control_mode = 0;
      previous_level = 255; 
    }
    /* Branch 3: DoIP Request Decoder (Ver 0x02, Inv 0xFD, Type 0x80 0x01) */
    else if (p->len >= 10 && payload[0] == 0x02 && payload[1] == 0xFD && payload[2] == 0x80 && payload[3] == 0x01)
    {
      uint8_t uds_service = payload[8];
      uint8_t uds_subfunc = payload[9];

      TxHeader.StdId = CAN_ID_GATEWAY_UDS_TX; 
      TxHeader.DLC = 8;
      
      TxData[0] = 0x02;        
      TxData[1] = uds_service; 
      TxData[2] = uds_subfunc; 
      TxData[3] = 0x55;        
      TxData[4] = 0x55; 
      TxData[5] = 0x55; 
      TxData[6] = 0x55; 
      TxData[7] = 0x55;

      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
          HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
      }
      HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
      
      printf("\r\n[GATEWAY] GIAI MA DoIP THANH CONG! Phat UDS %02X %02X xuong Node B.\r\n", uds_service, uds_subfunc);
    }

    pbuf_free(p); 
  }
}

/**
 * @brief  Initializes the UDP Listener for incoming PC commands.
 *
 * @return None
 */
void Gateway_UDP_Receiver_Init(void)
{
  struct udp_pcb *upcb = udp_new();
  if (upcb != NULL)
  {
    if (udp_bind(upcb, IP_ADDR_ANY, GATEWAY_UDP_PORT) == ERR_OK)
    {
      udp_recv(upcb, udp_receive_callback, NULL);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
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
