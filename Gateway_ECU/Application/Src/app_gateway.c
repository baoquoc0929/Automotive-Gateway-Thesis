/**
 * @file       app_gateway.c
 * @copyright  Copyright (C) 2026. All rights reserved.
 * @version    1.0.0
 * @date       2026-07-23
 * @author     Quoc Bao
 *             
 * @brief      Implementation of gateway routing between Ethernet and CAN.
 */

/* Includes ----------------------------------------------------------- */
#include "app_gateway.h"
#include "app_telemetry.h"
#include "main.h"
#include "can.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
#define EVT_UDS_RX_BIT     0x00000001U

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
extern CAN_HandleTypeDef   hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t             TxData[8];
extern uint32_t            TxMailbox;

extern osMessageQueueId_t  CAN_Data_QueueHandle;
extern osMessageQueueId_t  Eth_Data_QueueHandle;
extern osEventFlagsId_t    UdsEventFlagsHandle;

extern volatile uint32_t   can_sensor_start_time;
extern volatile uint32_t   can_uds_start_time;
extern uint8_t             gateway_uds_level;

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
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

void Gateway_Send_UDP(uint16_t dist_cm)
{
  char msg_buffer[50];
  sprintf(msg_buffer, "Dist: %d cm\r\n", dist_cm);
  UDP_Send_String(msg_buffer);
}

void Gateway_Send_UDS_UDP(uint8_t level)
{
  char msg_buffer[50];
  sprintf(msg_buffer, "UDS_ACK:%d\r\n", level); 
  UDP_Send_String(msg_buffer);
}

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

void App_Gateway_Logic_Task_Run(void)
{
  uint16_t received_dist;
  uint32_t flags;
  uint32_t end_time;
  float latency_us;
    
  /* Block and yield CPU until CAN data is available in the Queue */
  if (osMessageQueueGet(CAN_Data_QueueHandle, &received_dist, NULL, 0) == osOK)
  {
    /* 1. Process AUTO mode logic */
    if (control_mode == 0)
    {
      /* Calculate warning level and update filtered_distance */
      App_Telemetry_Handle_Distance(received_dist); 

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
    osMessageQueuePut(Eth_Data_QueueHandle, &current_distance, 0, 0);

    /* 3. Measure latency for CAN message from Node A */
    end_time = DWT->CYCCNT;
    latency_us = (float)(end_time - can_sensor_start_time) / (SystemCoreClock / 1000000.0f);
    static uint32_t sensor_print_counter = 0;
    sensor_print_counter++;
    if (sensor_print_counter >= 100) 
    {
      printf("[LATENCY - SENSOR] CAN -> Eth Queue: %.2f us\r\n", latency_us);
      sensor_print_counter = 0;
    }
  }
      
  /* 4. Check for incoming UDS response using Event Flags (Non-blocking check) */
  /* Check the event flags */
  flags = osEventFlagsWait(UdsEventFlagsHandle, EVT_UDS_RX_BIT, osFlagsWaitAny, 0);
  if (flags == EVT_UDS_RX_BIT)
  {
    /* Send UDS response via UDP */   
    Gateway_Send_UDS_UDP(gateway_uds_level);

    /* Measure latency for UDS response from Node B */
    end_time = DWT->CYCCNT;
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

void App_Gateway_Eth_Task_Run(void)
{
  uint16_t dist_to_send;
    
  /* 1. Check Manual Mode Timeout */
  if (control_mode == 1) 
  {
    if (HAL_GetTick() - last_manual_time > MANUAL_TIMEOUT) 
    {
      control_mode = 0; 
      previous_level = 255;     /* Force level update in the next cycle */
    }
  }

  /* 2. Wait for distance data from Logic Task and send via UDP */
  if (osMessageQueueGet(Eth_Data_QueueHandle, &dist_to_send, NULL, 10) == osOK)
  {
    Gateway_Send_UDP(dist_to_send);
  }
    
  osDelay(10); /* Yield CPU to other tasks (10ms tick) */
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
