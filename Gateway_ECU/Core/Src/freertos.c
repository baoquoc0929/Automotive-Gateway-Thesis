/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>     /* Required for printf */
#include <string.h>    /* Required for strlen */
#include "can.h"       /* Required for CAN handles and structures */
#include "lwip.h"      /* Required for LwIP definitions */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EVT_UDS_RX_BIT   0x00000001U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* External CAN Variables */
extern CAN_HandleTypeDef   hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t             TxData[8];
extern uint32_t            TxMailbox;

/* External System Logic Variables */
extern uint8_t             control_mode;
extern uint32_t            last_manual_time;
extern const uint32_t      MANUAL_TIMEOUT;
extern uint8_t             previous_level;
extern uint8_t             current_level;
extern uint16_t            filtered_distance;

/* External UDS Variables */
extern uint8_t             gateway_uds_level;

/* External Functions */
extern void Handle_Distance(uint16_t dist);
extern void Gateway_Send_UDP(uint16_t dist_cm);
extern void Gateway_UDP_Receiver_Init(void);
extern void Gateway_Send_UDS_UDP(uint8_t level);

/* Event Flags for UDS Handling */
osEventFlagsId_t UdsEventFlagsHandle;
const osEventFlagsAttr_t UdsEventFlags_attributes = {
  .name = "UdsEventFlags"
};

/* USER CODE END Variables */
/* Definitions for Logic_Task */
osThreadId_t Logic_TaskHandle;
const osThreadAttr_t Logic_Task_attributes = {
  .name = "Logic_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Eth_Task */
osThreadId_t Eth_TaskHandle;
const osThreadAttr_t Eth_Task_attributes = {
  .name = "Eth_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CAN_Data_Queue */
osMessageQueueId_t CAN_Data_QueueHandle;
const osMessageQueueAttr_t CAN_Data_Queue_attributes = {
  .name = "CAN_Data_Queue"
};
/* Definitions for Eth_Data_Queue */
osMessageQueueId_t Eth_Data_QueueHandle;
const osMessageQueueAttr_t Eth_Data_Queue_attributes = {
  .name = "Eth_Data_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLogicTask(void *argument);
void StartEthTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN_Data_Queue */
  CAN_Data_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &CAN_Data_Queue_attributes);

  /* creation of Eth_Data_Queue */
  Eth_Data_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &Eth_Data_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Logic_Task */
  Logic_TaskHandle = osThreadNew(StartLogicTask, NULL, &Logic_Task_attributes);

  /* creation of Eth_Task */
  Eth_TaskHandle = osThreadNew(StartEthTask, NULL, &Eth_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Create event flags for UDS handling */
  UdsEventFlagsHandle = osEventFlagsNew(&UdsEventFlags_attributes);
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLogicTask */
/**
  * @brief  Function implementing the Logic_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartLogicTask */
  
  /* Initialize UDP Receiver AFTER LwIP core has fully started */
  Gateway_UDP_Receiver_Init();

  /* Enable DWT for cycle counting to measure CAN message latency */
  DWT_ENABLE();
    
  uint16_t received_dist;
  uint32_t flags;

  uint32_t end_time;
  float latency_us;
    
  /* Infinite loop */
  for(;;)
  {
    /* Block and yield CPU until CAN data is available in the Queue */
    if (osMessageQueueGet(CAN_Data_QueueHandle, &received_dist, NULL, 0) == osOK)
    {
      /* 1. Process AUTO mode logic */
      if (control_mode == 0)
      {
        /* Calculate warning level and update filtered_distance */
        Handle_Distance(received_dist); 

        /* Send CAN message to Node B only if the warning level has changed */
        if (current_level != previous_level)
        {
          TxHeader.StdId = 0x450;
          TxHeader.DLC = 1;
          TxData[0] = current_level;
          
          if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
          {
            HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
          }
          
          if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
          {
            previous_level = current_level;
          }
        }
      }

      /* 2. Forward the filtered distance to the Ethernet Task */
      osMessageQueuePut(Eth_Data_QueueHandle, &filtered_distance, 0, 0);

      /* 3. Measure latency for CAN message from Node A */
      end_time = DWT_GET();
      latency_us = (float)(end_time - can_sensor_start_time) / (SystemCoreClock / 1000000.0f);
      static uint32_t sensor_print_counter = 0;
      sensor_print_counter++;
      if (sensor_print_counter >= 100) 
      {
          printf("[LATENCY - SENSOR] CAN -> Eth Queue: %.2f us\r\n", latency_us);
          sensor_print_counter = 0; // Reset counter
      }
    }
        
    /* 3. Check for incoming UDS response using Event Flags (Non-blocking check) */
    /* Check the event flags */
    flags = osEventFlagsWait(UdsEventFlagsHandle, EVT_UDS_RX_BIT, osFlagsWaitAny, 0);
    if (flags == EVT_UDS_RX_BIT)
    {   
      /* Send UDS response via UDP */
      Gateway_Send_UDS_UDP(gateway_uds_level);

      /* Measure latency for UDS response from Node B */
      end_time = DWT_GET();
      latency_us = (float)(end_time - can_uds_start_time) / (SystemCoreClock / 1000000.0f);
      printf("[LATENCY - UDS] CAN -> Eth UDP: %.2f us\r\n", latency_us);
			
      /* Print gateway information */
			printf("\r\n=======================================\r\n");
      printf("[GATEWAY] BAT DUOC PHAN HOI UDS TU NODE B!\r\n");
      printf("-> ID: 0x7E8 | Muc canh bao nhan duoc: %d\r\n", gateway_uds_level);
      printf("[GATEWAY] Dang gui goi tin Ethernet len PC...\r\n");
      printf("=======================================\r\n");
    }
    osDelay(1);   // Yield CPU to other tasks (1ms tick)
  }
  /* USER CODE END StartLogicTask */
}

/* USER CODE BEGIN Header_StartEthTask */
/**
* @brief Function implementing the Eth_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEthTask */
void StartEthTask(void *argument)
{
  /* USER CODE BEGIN StartEthTask */
  uint16_t dist_to_send;
    
  /* Infinite loop */
  for(;;)
  {
    /* 1. Check Manual Mode Timeout */
    if (control_mode == 1) 
    {
      if (HAL_GetTick() - last_manual_time > MANUAL_TIMEOUT) 
      {
        control_mode = 0; 
        previous_level = 255; /* Force level update in the next cycle */
      }
    }

    /* 2. Wait for distance data from Logic Task and send via UDP */
    if (osMessageQueueGet(Eth_Data_QueueHandle, &dist_to_send, NULL, 10) == osOK)
    {
        Gateway_Send_UDP(dist_to_send);
    }
    
    osDelay(10); /* Yield CPU to other tasks (10ms tick) */
  }
  /* USER CODE END StartEthTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

