/**
 * @file       hcsr04.c
 * @copyright  Copyright (C) 2026 Hong Quoc Bao. All rights reserved.
 * @license    Proprietary. For educational purposes only.
 * @version    1.0.0
 * @date       2026-01-28
 * @author     Hong Quoc Bao (baoquocbao2004@gmail.com)
 *
 * @brief      Implementation file for the HC-SR04 ultrasonic sensor driver.
 *
 * @note       This driver is designed for STM32 HAL-based projects.
 *
 */
 
/* Includes ----------------------------------------------------------- */
#include "hcsr04.h"

/* Private defines ---------------------------------------------------- */
#define TRIG_PULSE_DURATION_US  (10)      /*!< Duration of the trigger pulse in microseconds. */
#define SENSOR_TIMEOUT_MS       (50)      /*!< Timeout in milliseconds for waiting for an echo response. */
#define SOUND_SPEED_CM_PER_US   (0.0343f) /*!< Speed of sound in cm/µs at room temperature. */

/* Private enumerate/structure ---------------------------------------- */
// No private enums or structures needed for this driver.

/* Private macros ----------------------------------------------------- */
// No private macros needed for this driver.

/* Public variables --------------------------------------------------- */
// No public variables are exposed by this driver.

/* Private variables -------------------------------------------------- */
static TIM_HandleTypeDef *g_timer_handle;         /*!< Pointer to the timer handle used for measurements. */
static volatile uint32_t g_echo_start_time   = 0; /*!< Timestamp when the echo pulse starts (rising edge). */
static volatile uint32_t g_echo_end_time     = 0; /*!< Timestamp when the echo pulse ends (falling edge). */
static volatile uint8_t  g_is_echo_captured  = 0; /*!< Flag to indicate that a full echo pulse has been captured. */

/* Private function prototypes ---------------------------------------- */
/**
 * @brief  Generates a 10µs pulse on the TRIG pin to start a measurement.
 *
 * @param  None
 *
 * @return None
 */
static void hcsr04_trigger(void);

/* Function definitions ----------------------------------------------- */
void HCSR04_Init(TIM_HandleTypeDef *htim)
{
  g_timer_handle = htim;
  HAL_TIM_Base_Start(g_timer_handle);
}

float HCSR04_Read(void)
{
  uint32_t timeout_start;
  uint32_t echo_duration;
  float distance = -1.0f; // Default to error value

  // 1. Trigger the sensor
  hcsr04_trigger();
  g_is_echo_captured = 0; // Reset the capture flag

  // 2. Wait for the echo to be captured by the ISR, with a timeout
  timeout_start = HAL_GetTick();
  while (!g_is_echo_captured)
  {
    if (HAL_GetTick() - timeout_start > SENSOR_TIMEOUT_MS)
    {
      return -1.0f; // Return error on timeout
    }
  }

  // 3. Calculate the duration of the echo pulse
  if (g_echo_end_time >= g_echo_start_time)
  {
    echo_duration = g_echo_end_time - g_echo_start_time;
  }
  else // Handle timer counter overflow
  {
    // Assuming a 16-bit timer (max value 65535)
    echo_duration = (0xFFFF - g_echo_start_time) + g_echo_end_time + 1;
  }

  // 4. Calculate distance in cm
  // Distance = (Time * Speed of Sound) / 2
  distance = (float)echo_duration * SOUND_SPEED_CM_PER_US / 2.0f;

  return distance;
}

void HCSR04_EXTI_Callback(void)
{
  if (HAL_GPIO_ReadPin(SR04_Echo_GPIO_Port, SR04_Echo_Pin) == GPIO_PIN_SET)
  {
    // Rising edge: capture the start time
    g_echo_start_time = __HAL_TIM_GET_COUNTER(g_timer_handle);
  }
  else // Falling edge
  {
    // Falling edge: capture the end time and set the flag
    g_echo_end_time = __HAL_TIM_GET_COUNTER(g_timer_handle);
    g_is_echo_captured = 1;
  }
}

/* Private function definitions ----------------------------------------------- */
static void hcsr04_trigger(void)
{
  // Set TRIG pin high
  HAL_GPIO_WritePin(SR04_Trig_GPIO_Port, SR04_Trig_Pin, GPIO_PIN_SET);

  // Wait for 10 microseconds using the timer for better accuracy
  __HAL_TIM_SET_COUNTER(g_timer_handle, 0);
  while (__HAL_TIM_GET_COUNTER(g_timer_handle) < TRIG_PULSE_DURATION_US);

  // Set TRIG pin low
  HAL_GPIO_WritePin(SR04_Trig_GPIO_Port, SR04_Trig_Pin, GPIO_PIN_RESET);
}

/* End of file -------------------------------------------------------- */
