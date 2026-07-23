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
#include <stdio.h>        /* Required for printf */
#include <string.h>       /* Required for strlen */
#include "can.h"          /* Required for CAN handles and structures */
#include "lwip.h"         /* Required for LwIP definitions */
#include "app_gateway.h"  /* Required for gateway routing functions */

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
/* Definitions for UDS Event Flag */
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
    
  /* Infinite loop */
  for(;;)
  {
    App_Gateway_Logic_Task_Run();  /* Core logic for CAN to Ethernet routing and UDS handling */
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
  
  /* Infinite loop */
  for(;;)
  {
    App_Gateway_Eth_Task_Run();  /* Core logic for Ethernet transmission and manual mode timeout */
  }
  /* USER CODE END StartEthTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

