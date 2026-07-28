/**
 * @file       app_gateway.h
 * @copyright  Copyright (C) 2026. All rights reserved.
 * @version    1.0.0
 * @date       2026-07-23
 * @author     Quoc Bao
 *             
 * @brief      Gateway routing and network communication module.
 */

/* Define to prevent recursive inclusion--------------------------- */
#ifndef __APP_GATEWAY_H
#define __APP_GATEWAY_H

/* Includes-------------------------------------------------------- */
#include <stdint.h>
#include "lwip/udp.h"

/* Public defines-------------------------------------------------- */
/* Network Configuration */
#define PC_DEST_IP_0            (192)
#define PC_DEST_IP_1            (168)
#define PC_DEST_IP_2            (1)
#define PC_DEST_IP_3            (100)
#define GATEWAY_UDP_PORT        (13400)

/* CAN Protocol IDs */
#define CAN_ID_NODE_A_RX        (0x250)
#define CAN_ID_GATEWAY_APP_TX   (0x450)
#define CAN_ID_GATEWAY_UDS_TX   (0x7E0)
#define CAN_ID_NODE_B_UDS_RX    (0x7E8)

/* DoIP ISO 13400 Constants */
#define DOIP_PROTOCOL_VER_2012      0x02
#define DOIP_INV_PROTOCOL_VER       0xFD
#define DOIP_PAYLOAD_TYPE_DIAG      0x8001

/* Logical Addresses */
#define DOIP_LOGICAL_GW_ADDR        0x1000  /* Gateway STM32 */
#define DOIP_LOGICAL_TESTER_ADDR    0x0E80  /* PC/Python Tester */

/* UDS ISO 14229 Services */
#define UDS_SID_ECU_RESET           0x11
#define UDS_SID_READ_DATA_BY_ID     0x22
#define UDS_SID_IO_CONTROL          0x2F
#define UDS_POSITIVE_RES_OFFSET     0x40

/* Data Identifiers (DID) */
#define DID_DISTANCE_SENSOR         0x0101

/* UDS IO Control Parameters (For 0x2F) */
#define UDS_IO_RETURN_CONTROL       0x00
#define UDS_IO_RESET_TO_DEFAULT     0x01
#define UDS_IO_FREEZE_STATE         0x02
#define UDS_IO_SHORT_TERM_ADJ       0x03

/* Data Identifiers (DID) */
#define DID_DISTANCE_SENSOR         0x0101
#define DID_ACTUATOR_CONTROL        0x0102  /* Actuator Control DID */

/* ISO 15765-2 CAN Transport Protocol Constants */
#define CAN_TP_PCI_SINGLE_FRAME_2B  0x02  /* Single Frame, DLC = 2 */
#define CAN_TP_PADDING_VALUE        0x55  /* Standard padding for unused CAN payload */

/* Public enumerate/structure-------------------------------------- */
/* Public macros--------------------------------------------------- */
/* Public variables------------------------------------------------ */

/* Public function prototypes-------------------------------------- */

/**
 * @brief  Helper function to send a raw byte array via UDP.
 *
 * @param[in]  payload  Pointer to the byte array to be sent.
 * @param[in]  length   The number of bytes to send.
 *
 * @return None
 */
void UDP_Send_DoIP_Raw(uint8_t *payload, uint16_t length);

/**
 * @brief  Constructs and transmits a UDS Positive Response over DoIP in raw hex format.
 *
 * This function processes the requested Service ID, calculates the corresponding 
 * Positive Response SID (+ 0x40), and encapsulates the payload within a standard 
 * DoIP diagnostic message (ISO 13400). It automatically inserts the routing 
 * addresses (SA: Gateway, TA: Tester) and the 8-byte DoIP header.
 *
 * @param[in]  request_sid  The requested UDS Service ID (e.g., 0x11, 0x22, 0x2F) to be acknowledged.
 * @param[in]  data         The 16-bit data value to append to the payload (specifically used for service 0x22).
 *
 * @return None
 */
void Gateway_Send_UDS_Response_Hex(uint8_t request_sid, uint16_t data);

/**
 * @brief  Constructs a DoIP Diagnostic Message containing distance data as payload.
 *
 * @param[in]  dist_cm  The distance value to be transmitted.
 *
 * @return None
 */
void Gateway_Send_DoIP_Data(uint16_t dist_cm);

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
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port);

/**
 * @brief  Initializes the UDP Listener for incoming PC commands.
 *
 * @return None
 */
void Gateway_UDP_Receiver_Init(void);

/**
 * @brief  Executes the core logic for the Gateway RTOS task.
 *         Responsibilities include: fetching CAN sensor data from the queue, 
 *         evaluating safety levels via the Telemetry module, routing UDS 
 *         responses to the Ethernet queue, and measuring system latency.
 *
 * @attention This function is non-blocking (except for event waits with timeout 0) 
 *            and must be called periodically inside the StartLogicTask infinite loop.
 *
 * @return None
 */
void App_Gateway_Logic_Task_Run(void);

/**
 * @brief  Executes the network communication handling for the Ethernet RTOS task.
 *         Responsibilities include: monitoring the MANUAL mode timeout to safely 
 *         fallback to AUTO mode, and transmitting queued telemetry data to the 
 *         PC via UDP.
 *
 * @attention This function must be called periodically inside the StartEthTask 
 *            infinite loop.
 *
 * @return None
 */
void App_Gateway_Eth_Task_Run(void);

#endif // __APP_GATEWAY_H

/* End of file----------------------------------------------------- */
