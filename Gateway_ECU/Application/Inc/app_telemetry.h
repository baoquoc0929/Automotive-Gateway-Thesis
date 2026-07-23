/**
 * @file       app_telemetry.h
 * @copyright  Copyright (C) 2026. All rights reserved.
 * @version    1.0.0
 * @date       2026-07-23
 * @author     Bao Quoc
 *             
 * @brief      Telemetry and safety logic module for Gateway ECU.         
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __APP_TELEMETRY_H
#define __APP_TELEMETRY_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Enumerated type for warning levels.
 */
typedef enum {
    LEVEL_SAFE,         /**< Safe distance */
    LEVEL_CAUTION,      /**< Cautionary distance */
    LEVEL_WARNING,      /**< Warning distance */
    LEVEL_DANGER        /**< Dangerous distance */
} WarningLevel_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */
/* Telemetry & System Logic Variables */
extern uint16_t           current_distance;  /**< Distance received from Node A (cm) */
extern WarningLevel_t     current_level;     /**< Calculated warning level */
extern uint8_t            previous_level;    /**< Memory to detect level changes */

extern uint8_t            control_mode;      /**< 0: AUTO (Sensor), 1: MANUAL (PC-based) */
extern uint32_t           last_manual_time;  /**< Timestamp of last received PC command */
extern const uint32_t     MANUAL_TIMEOUT;    /**< 5 seconds safety timeout for manual mode */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Evaluate warning level using a smoothed distance value.
 *
 * @param[in]  dist  Raw distance in cm from sensor.
 */
void App_Telemetry_Handle_Distance(uint16_t dist);

#endif // __APP_TELEMETRY_H

/* End of file -------------------------------------------------------- */
