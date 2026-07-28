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
extern uint16_t            current_distance;

uint8_t last_uds_request_sid = 0x00;    

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void UDP_Send_DoIP_Raw(uint8_t *payload, uint16_t length)
{
  struct udp_pcb *upcb = udp_new();
  if (upcb != NULL)
  {
    ip_addr_t DestIPaddr;
    
    /* Configure the destination IP address (PC target) */
    IP4_ADDR(&DestIPaddr, PC_DEST_IP_0, PC_DEST_IP_1, PC_DEST_IP_2, PC_DEST_IP_3); 

    /* Allocate memory for the packet based on the specified length */
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);  
    if (p != NULL)
    {
      /* Copy the payload data into the allocated pbuf */
      pbuf_take(p, payload, length);
      
      /* Transmit the packet to the destination IP over DoIP port 13400 */
      udp_sendto(upcb, p, &DestIPaddr, GATEWAY_UDP_PORT); 
      
      /* Free the pbuf to prevent memory leaks */
      pbuf_free(p);
    }
    
    /* Remove the UDP protocol control block to free up resources */
    udp_remove(upcb); 
  }
}

void Gateway_Send_UDS_Response_Hex(uint8_t request_sid, uint16_t data)
{
  uint8_t doip_packet[60];
  uint16_t uds_len = 0;
  
  /* Calculate Positive Response (Add 0x40 to Service ID) */
  uint8_t response_sid = request_sid + UDS_POSITIVE_RES_OFFSET; 
  
  /* Build standard UDS Payload (Starting at index 12) */
  if (request_sid == UDS_SID_READ_DATA_BY_ID)
  {
    /* Read Data By Identifier (0x22) -> Response: 0x62 */
    doip_packet[12] = response_sid; 
    doip_packet[13] = (DID_DISTANCE_SENSOR >> 8) & 0xFF;  /* DID Byte High */
    doip_packet[14] = DID_DISTANCE_SENSOR & 0xFF;         /* DID Byte 2 */
    doip_packet[15] = (data >> 8) & 0xFF;                 /* Distance Data (MSB - Most Significant Byte) */
    doip_packet[16] = data & 0xFF;                        /* Distance Data (LSB - Least Significant Byte) */
    uds_len = 5;
  } 
  else if (request_sid == UDS_SID_ECU_RESET)
  {
    /* ECU Reset (0x11) -> Response: 0x51 */
    doip_packet[12] = response_sid; 
    doip_packet[13] = 0x01;               /* Reset Type (e.g., 0x01 Hard Reset) */
    uds_len = 2;
  }
  else if (request_sid == UDS_SID_IO_CONTROL)
  {
    /* Input Output Control By Identifier (0x2F) -> Response: 0x6F */
    doip_packet[12] = response_sid; 
    doip_packet[13] = (DID_DISTANCE_SENSOR >> 8) & 0xFF;  /* DID Byte 1 (0x0101) */
    doip_packet[14] = DID_DISTANCE_SENSOR & 0xFF;         /* DID Byte 2 */
    doip_packet[15] = UDS_IO_SHORT_TERM_ADJ;
    uds_len = 4;
  }

  /* Insert DoIP network addresses (4 bytes: SA and TA) at indices 8 to 11 */
  /* Note: For a response, Gateway is Source (0x1000) and Tester is Target (0x0E80) */
  doip_packet[8]  = (DOIP_LOGICAL_GW_ADDR >> 8) & 0xFF;       /* Source Address Byte high */
  doip_packet[9]  = DOIP_LOGICAL_GW_ADDR & 0xFF;              /* Source Address Byte low */
  doip_packet[10] = (DOIP_LOGICAL_TESTER_ADDR >> 8) & 0xFF;   /* Target Address Byte high */
  doip_packet[11] = DOIP_LOGICAL_TESTER_ADDR & 0xFF;          /* Target Address Byte low */
  
  /* Pack the 8-byte DoIP header */
  uint16_t payload_len = 4 + uds_len;           /* 4 bytes Address + UDS payload length */
  uint16_t total_len = 8 + payload_len;         /* Total packet length to transmit */
  
  doip_packet[0] = DOIP_PROTOCOL_VER_2012;                    /* Protocol Version: ISO 13400-2 (2012) */
  doip_packet[1] = DOIP_INV_PROTOCOL_VER;                     /* Inverse Protocol Version */
  doip_packet[2] = (DOIP_PAYLOAD_TYPE_DIAG >> 8) & 0xFF;      /* Payload Type: Diagnostic Message (MSB) */
  doip_packet[3] = DOIP_PAYLOAD_TYPE_DIAG & 0xFF;             /* Payload Type: Diagnostic Message (LSB) */
  doip_packet[4] = (payload_len >> 24) & 0xFF;                /* Payload Length (Byte 1 - MSB) */
  doip_packet[5] = (payload_len >> 16) & 0xFF;                /* Payload Length (Byte 2) */
  doip_packet[6] = (payload_len >> 8) & 0xFF;                 /* Payload Length (Byte 3) */
  doip_packet[7] = payload_len & 0xFF;                        /* Payload Length (Byte 4 - LSB) */

  /* Transmit the packet via LwIP */
  UDP_Send_DoIP_Raw(doip_packet, total_len);
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  if (p != NULL)
  {
    uint8_t *payload = (uint8_t *)p->payload;

    /* SINGLE THREAD: DoIP Request Decoder strictly compliant with ISO 13400 */
    if (p->len >= 14 && 
        payload[0] == DOIP_PROTOCOL_VER_2012 && 
        payload[1] == DOIP_INV_PROTOCOL_VER && 
        payload[2] == ((DOIP_PAYLOAD_TYPE_DIAG >> 8) & 0xFF) && 
        payload[3] == (DOIP_PAYLOAD_TYPE_DIAG & 0xFF))
    {
      uint8_t uds_service = payload[12];
      uint8_t uds_subfunc = payload[13]; 
      
      last_uds_request_sid = uds_service;

      /* HANDLE 0x22 SERVICE (Read Data By Identifier) - Gateway Caching */
      if (uds_service == UDS_SID_READ_DATA_BY_ID)
      {
        Gateway_Send_UDS_Response_Hex(uds_service, current_distance);
        printf("\r\n[GATEWAY] UDS 0x22 CACHE HIT! Returning distance: %d cm\r\n", current_distance);
      }
      
      /* HANDLE 0x2F SERVICE (Input Output Control) - UDS AUTO/MANUAL SWITCH */
      else if (uds_service == UDS_SID_IO_CONTROL)
      {
        uint16_t request_did = (payload[13] << 8) | payload[14];
        uint8_t control_option = payload[15]; // 0x00 (Auto) or 0x03 (Manual)

        if (request_did == DID_ACTUATOR_CONTROL) 
        {
           if (control_option == UDS_IO_RETURN_CONTROL) /* Replaces legacy 'A' or 'a' character reception */
           {
             control_mode = 0;
             previous_level = 255;
             printf("\r\n[GATEWAY] UDS 0x2F: Return Control to ECU -> AUTO Mode.\r\n");
           }
           else if (control_option == UDS_IO_SHORT_TERM_ADJ) /* Replaces legacy '0'-'3' character reception */
           {
             /* 
              * According to ISO 14229, the Control Option (0x03) is followed by the ControlState byte.
              * This byte is located at index 16 of the DoIP frame.
              */
             uint8_t control_state = payload[16]; 
             
             if (control_state <= 3) 
             {
               control_mode = 1; 
               last_manual_time = HAL_GetTick(); 
                     
               /* Forward directly to the CAN network */
               TxHeader.StdId = CAN_ID_GATEWAY_APP_TX;
               TxHeader.DLC = 1;
               TxData[0] = control_state;
               
               if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
                   HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
               }
               HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
               
               printf("\r\n[GATEWAY] UDS 0x2F: Short Term Adj -> MANUAL Mode set to %d.\r\n", control_state);
             }
           }
           
           /* Gateway must send a 0x6F positive response back to Tester to prevent Wireshark malformed errors */
           Gateway_Send_UDS_Response_Hex(uds_service, request_did); 
        }
      }
      
      /* FORWARD OTHER SERVICES (e.g., 0x11 ECU Reset) TO NODE B VIA CAN */
      else
      {
        TxHeader.StdId = CAN_ID_GATEWAY_UDS_TX; 
        TxHeader.DLC = 8;
      
        TxData[0] = CAN_TP_PCI_SINGLE_FRAME_2B;  /* ISO 15765-2 compliance */
        TxData[1] = uds_service; 
        TxData[2] = uds_subfunc; 
        TxData[3] = CAN_TP_PADDING_VALUE;        /* 0x55 Frame Padding */
        TxData[4] = CAN_TP_PADDING_VALUE; 
        TxData[5] = CAN_TP_PADDING_VALUE; 
        TxData[6] = CAN_TP_PADDING_VALUE; 
        TxData[7] = CAN_TP_PADDING_VALUE;

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
          HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
      
        printf("\r\n[GATEWAY] DoIP DECODED! Forwarding UDS %02X %02X to Node B.\r\n", uds_service, uds_subfunc);
      }
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
    
  /* Non-blocking check for incoming CAN sensor data in the Queue */
  if (osMessageQueueGet(CAN_Data_QueueHandle, &received_dist, NULL, 0) == osOK)
  {
    /* --- PROCESS AUTO MODE LOGIC --- */
    if (control_mode == 0)
    {
      /* Calculate warning level and update filtered_distance */
      App_Telemetry_Handle_Distance(received_dist); 

      /* Transmit CAN message to Actuator Node only upon state transition */
      if (current_level != previous_level)
      {
        TxHeader.StdId = CAN_ID_GATEWAY_APP_TX;
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

    /* Forward the filtered data to Ethernet Task for UI monitoring */
    osMessageQueuePut(Eth_Data_QueueHandle, &current_distance, 0, 0);

    /* --- MEASURE LATENCY: SENSOR DATA ROUTING --- */
    end_time = DWT->CYCCNT;
    latency_us = (float)(end_time - can_sensor_start_time) / (SystemCoreClock / 1000000.0f);
    
    static uint32_t sensor_print_counter = 0;
    sensor_print_counter++;
    if (sensor_print_counter >= 100) 
    {
      //printf("[LATENCY - SENSOR] CAN -> Eth Queue: %.2f us\r\n", latency_us);
      sensor_print_counter = 0;
    }
  }
      
  /* Non-blocking check for incoming UDS diagnostic responses */
  flags = osEventFlagsWait(UdsEventFlagsHandle, EVT_UDS_RX_BIT, osFlagsWaitAny, 0);
  
  /* Validate that the exact flag was returned (ignoring CMSIS error codes with MSB set) */
  if (flags == EVT_UDS_RX_BIT)
  {
    /* Transmit the DoIP-formatted UDS response via UDP to the Tester */   
    Gateway_Send_UDS_Response_Hex(last_uds_request_sid, 0);

    /* --- MEASURE LATENCY: UDS DIAGNOSTIC ROUTING --- */
    end_time = DWT->CYCCNT;
    latency_us = (float)(end_time - can_uds_start_time) / (SystemCoreClock / 1000000.0f);
    printf("[LATENCY - UDS] CAN -> Eth UDP: %.2f us\r\n", latency_us);
    
    /* Print Gateway diagnostic trace */
    printf("\r\n=======================================\r\n");
    printf("[GATEWAY] UDS RESPONSE RECEIVED FROM TARGET ECU!\r\n");
    printf("-> ECU ID: 0x7E8 | Diagnostic Status: %d\r\n", gateway_uds_level);
    printf("[GATEWAY] Forwarding DoIP Packet to PC Tester...\r\n");
    printf("=======================================\r\n");
  }
  
  /* Yield CPU to allow other RTOS tasks to execute (1ms tick interval) */
  osDelay(1);   
}

void App_Gateway_Eth_Task_Run(void)
{
  uint16_t dist_to_send;
    
  /* --- FAIL-SAFE MECHANISM: MANUAL MODE TIMEOUT --- */
  /* Automatically returns control to ECU (Auto Mode) if tester communication is lost or idle */
  if (control_mode == 1) 
  {
    if (HAL_GetTick() - last_manual_time > MANUAL_TIMEOUT) 
    {
      control_mode = 0; 
      previous_level = 255;     /* Force a state evaluation in the next control cycle */
      
      /* Optional: Print a warning via UART for debugging */
      printf("\r\n[FAIL-SAFE] Manual control timeout! Restored to AUTO mode.\r\n");
    }
  }

  /* --- ETHERNET TELEMETRY / DIAGNOSTIC STREAMING --- */
  /* Wait for filtered distance data from Logic Task with a 10ms timeout */
  if (osMessageQueueGet(Eth_Data_QueueHandle, &dist_to_send, NULL, 10) == osOK)
  {
    /* 
     * NOTE: Using UDS Read Data (0x22) for continuous streaming is a workaround 
     * for the UI real-time graph. In a strict ISO 14229 implementation, 
     * Service 0x2A (Read Data By Periodic Identifier) should be considered.
     */
    Gateway_Send_UDS_Response_Hex(UDS_SID_READ_DATA_BY_ID, dist_to_send);
  }
    
  /* Yield CPU to allow other FreeRTOS tasks to execute */
  osDelay(10); 
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
