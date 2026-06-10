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
extern volatile uint8_t    gateway_uds_flag;
extern uint8_t             gateway_uds_level;

/* External Functions */
extern void Handle_Distance(uint16_t dist);
extern void Gateway_Send_UDP(uint16_t dist_cm);
extern void Gateway_UDP_Receiver_Init(void);
extern void Gateway_Send_UDS_UDP(uint8_t level);

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
  /* add events, ... */
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
    
  uint16_t received_dist;
    
  /* Infinite loop */
  for(;;)
  {
    /* Block and yield CPU until CAN data is available in the Queue */
    if (osMessageQueueGet(CAN_Data_QueueHandle, &received_dist, NULL, osWaitForever) == osOK)
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
    }
        
    /* 3. Check for incoming UDS response from Node B */
    if (gateway_uds_flag == 1)
    {
        gateway_uds_flag = 0; /* Clear flag immediately */
      
        printf("\r\n=======================================\r\n");
        printf("[GATEWAY] BAT DUOC PHAN HOI UDS TU NODE B!\r\n");
        printf("-> ID: 0x7E8 | Muc canh bao nhan duoc: %d\r\n", gateway_uds_level);
        printf("[GATEWAY] Dang gui goi tin Ethernet len PC...\r\n");
        printf("=======================================\r\n");
            
        Gateway_Send_UDS_UDP(gateway_uds_level);
    }
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

