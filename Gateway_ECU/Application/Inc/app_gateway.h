/**
 * @file       app_gateway.h
 * @copyright  Copyright (C) 2026. All rights reserved.
 * @version    1.0.0
 * @date       2026-07-23
 * @author     Quoc Bao
 *             
 * @brief      Gateway routing and network communication module.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __APP_GATEWAY_H
#define __APP_GATEWAY_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include "lwip/udp.h"

/* Public defines ----------------------------------------------------- */
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

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Helper function to send a raw string via UDP.
 *
 * @param[in]  msg_string  Pointer to the null-terminated string to send.
 *
 * @return None
 */
void UDP_Send_String(const char *msg_string);

/**
 * @brief  Formats and sends the distance telemetry via UDP to the PC.
 *
 * @param[in]  dist_cm  Distance value in centimeters.
 *
 * @return None
 */
void Gateway_Send_UDP(uint16_t dist_cm);

/**
 * @brief  Formats and sends the UDS acknowledgment level via UDP to the PC.
 *
 * @param[in]  level  The UDS response level received from Node B.
 *
 * @return None
 */
void Gateway_Send_UDS_UDP(uint8_t level);

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

/* End of file -------------------------------------------------------- */
