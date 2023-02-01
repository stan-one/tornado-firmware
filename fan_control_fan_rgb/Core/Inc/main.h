/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/**
 *  @mainpage The firmare of the pcb
  * @brief  The application entry point. Checks if there is any new incoming data, sets it and then sends back the current state of the system and toggles a led as a keep alive signal.
  * @retval int
  */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#include "config.h"

#define TEST_SIZE 60
typedef struct data_out{
	uint32_t rpm0;
	uint32_t rpm1;
	uint32_t rpm2;
	uint32_t rpm3;
	uint32_t rpm_pump;
	uint32_t temperature;
}data_out_t;

typedef struct data_in{
    uint32_t pwm_f0;
    uint32_t pwm_f1;
    uint32_t pwm_f2;
    uint32_t pwm_f3;
    uint8_t sw_1;
    uint8_t sw_2;
    int freq_pwm;
}data_in_t;

typedef struct pulse{
	uint32_t p1;
	uint32_t p2;
}collected_pulses_t[NUM_FANS];

typedef struct led_setup{
    uint32_t num_led_fan;
    uint32_t num_led_strip;
}led_setup_t;


enum fans{PUMP, FAN_1, FAN_2, FAN_3, FAN_4};
/*HARDWARE DEPENDENT FUNCTIONS*/
/**
 * @return Raw adc data
 */

uint32_t adc_read();


/***
 *
 *
 *
 *
 *
 */


/**
 * @brief Calcules the temperature of the board using the thermistor. Depends on the thermistor constants and on the voltage divider.
 * @param raw_adc
 * @return
 */
uint32_t convert_raw_to_temp(uint32_t raw_adc);
/**
 * @brief Function that picks up and stores the timer counter when a tachometer pulse happens.
 * stores the data inside the pulse set datatype
 * @param pulse_set
 */
void read_fan_pulses(collected_pulses_t pulse_set);
/**
 * @brief inits the pulse_set. Needed because might be not all 4 fans are present.
 * @param pulse_set
 */
void init_pulses(collected_pulses_t pulses);
/**
 * @brief From the collected pulses, this function calculates the speed at which the fans are spinning
 * @param speeds
 * @param pulses
 */

void fan_speed_calculator(uint32_t speeds[], collected_pulses_t  pulses);
/**
 * @brief Processes the raw data coming from the UI
 * @param buff
 * @param data_ui
 * @return
 */
uint8_t process_buffer_command(char buff[], data_in_t *data_ui);
/**
 * @brief Processes the set up data. How many leds are present in the fans,
 * @param buff
 * @param data_setup
 * @return
 */
uint8_t process_setup_data(char buff[], led_setup_t *data_setup);
/**
 * @brief Process the hole string coming from the UI. The desired led color for all the leds.
 * @param buff
 * @param effect
 * @return
 */
uint8_t process_led_effect_data(char buff[], uint32_t effect[]);
/**
 * @brief prepare the data ti be send to the leds using the DMA
 * @param pwm_data
 * @param effect
 * @param leds
 * @return
 */
uint32_t create_data_ws(uint16_t *pwm_data, uint32_t effect[], led_setup_t leds);
/**
 * @brief Process the data corresponding to one led
 * @param led_s
 * @return
 */
uint32_t fragment_led(char led_s[]);
/**
 * @brief set the speed in the data out structure
 * @param speeds
 * @param data_out
 */
void set_speeds_in_data(uint32_t speeds[], data_out_t *data_out);
/**
 * @brief Create the string to be send to the UI
 * @param msg
 * @param data
 */
void create_data_pcb(char* msg, data_out_t data);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_STATE_Pin GPIO_PIN_14
#define BOARD_STATE_GPIO_Port GPIOC
#define RGB_FAN_1_Pin GPIO_PIN_0
#define RGB_FAN_1_GPIO_Port GPIOA
#define THERMISTOR_Pin GPIO_PIN_1
#define THERMISTOR_GPIO_Port GPIOA
#define RGB_FAN_3_Pin GPIO_PIN_2
#define RGB_FAN_3_GPIO_Port GPIOA
#define RGB_FAN_4_Pin GPIO_PIN_3
#define RGB_FAN_4_GPIO_Port GPIOA
#define PWM_FAN_1_Pin GPIO_PIN_6
#define PWM_FAN_1_GPIO_Port GPIOA
#define PWM_FAN_2_Pin GPIO_PIN_7
#define PWM_FAN_2_GPIO_Port GPIOA
#define PWM_FAN_3_Pin GPIO_PIN_0
#define PWM_FAN_3_GPIO_Port GPIOB
#define PWM_FAN_4_Pin GPIO_PIN_1
#define PWM_FAN_4_GPIO_Port GPIOB
#define GEN_SWITCH_2_Pin GPIO_PIN_10
#define GEN_SWITCH_2_GPIO_Port GPIOB
#define GEN_SWITCH_1_Pin GPIO_PIN_11
#define GEN_SWITCH_1_GPIO_Port GPIOB
#define TACH_FAN_1_Pin GPIO_PIN_12
#define TACH_FAN_1_GPIO_Port GPIOB
#define TACH_FAN_1_EXTI_IRQn EXTI4_15_IRQn
#define TACH_FAN_2_Pin GPIO_PIN_13
#define TACH_FAN_2_GPIO_Port GPIOB
#define TACH_FAN_2_EXTI_IRQn EXTI4_15_IRQn
#define TACH_FAN_3_Pin GPIO_PIN_14
#define TACH_FAN_3_GPIO_Port GPIOB
#define TACH_FAN_3_EXTI_IRQn EXTI4_15_IRQn
#define TACH_FAN_4_Pin GPIO_PIN_15
#define TACH_FAN_4_GPIO_Port GPIOB
#define TACH_FAN_4_EXTI_IRQn EXTI4_15_IRQn
#define LED_STRIP_1_Pin GPIO_PIN_8
#define LED_STRIP_1_GPIO_Port GPIOA
#define RGB_FAN_2_Pin GPIO_PIN_3
#define RGB_FAN_2_GPIO_Port GPIOB
#define SWITCH_FAN_1_Pin GPIO_PIN_4
#define SWITCH_FAN_1_GPIO_Port GPIOB
#define SWITCH_FAN_2_Pin GPIO_PIN_5
#define SWITCH_FAN_2_GPIO_Port GPIOB
#define SWITCH_FAN_3_Pin GPIO_PIN_6
#define SWITCH_FAN_3_GPIO_Port GPIOB
#define SWITCH_FAN_4_Pin GPIO_PIN_7
#define SWITCH_FAN_4_GPIO_Port GPIOB
#define TACH_PUMP_Pin GPIO_PIN_9
#define TACH_PUMP_GPIO_Port GPIOB
#define TACH_PUMP_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
