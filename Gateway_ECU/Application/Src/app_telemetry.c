/**
 * @file       app_telemetry.c
 * @copyright  Copyright (C) 2026. All rights reserved.
 * @version    1.0.0
 * @date       2026-07-23
 * @author     Quoc Bao
 *             
 * @brief      Implementation of telemetry and safety level calculations.
 */

/* Includes ----------------------------------------------------------- */
#include "app_telemetry.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
uint16_t           current_distance = 0; 
WarningLevel_t     current_level = LEVEL_SAFE; 
uint8_t            previous_level = 255; 

uint8_t            control_mode = 0; 
uint32_t           last_manual_time = 0; 
const uint32_t     MANUAL_TIMEOUT = 5000;

/* Private variables -------------------------------------------------- */
static uint16_t    filtered_distance = 0; /**< Filtered distance state for moving average */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void App_Telemetry_Handle_Distance(uint16_t dist)
{
  /* Moving average filter: Retain 70% of previous value, blend with 30% new value */
  if(filtered_distance == 0)
  {
    filtered_distance = dist; 
  }
  
  filtered_distance = (filtered_distance * 7 + dist * 3) / 10;

  current_distance = filtered_distance;

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
/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
