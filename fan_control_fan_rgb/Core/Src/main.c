/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t pulse_source = 0;
uint8_t pulse_timout = 0;
uint8_t transfer_complete = 0;
uint8_t rgb_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/**
 * @brief Callback of the ISR for the tachometers of the fans
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin){
    	case TACH_PUMP_Pin:
    		pulse_source = TACH_PUMP;
    	break;
    	case TACH_FAN_1_Pin:
    		pulse_source = TACH_FAN_1;
    	break;
    	case TACH_FAN_2_Pin:
    		pulse_source = TACH_FAN_2;
    	break;
    	case TACH_FAN_3_Pin:
    		pulse_source = TACH_FAN_3;
    	break;
    	case TACH_FAN_4_Pin:
    		pulse_source = TACH_FAN_4;
    	break;
    }
}
/**
 * @brief ISR callback for the timer monitoring the timeout of the tachometers
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallbackck(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM14){
	 pulse_timout = 1;
 }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t buffer[USB_BUFFER_SIZE] = {0};
uint8_t buffer_ans[USB_BUFFER_SIZE] = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t adc_read(){
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 1);
    return HAL_ADC_GetValue(&hadc);
}


uint32_t convert_raw_to_temp(uint32_t raw_adc){
    uint32_t temperature = 0;
	if(0>raw_adc || raw_adc>ADC_RES){
		temperature = IMPOSSIBLE_VALUE_TEMP;
	}
    else{
        float real_voltage = ((float)raw_adc/ADC_RES)*MAX_VOLT;
        float Rth = (-real_voltage*R1)/(real_voltage-V_SUPPLY);
        float a = log(Rth/(float)RES_AT_25C)/(float)BETA_TERMISTOR;
        float b = (float)1/298;
        float c = 1/(a+b);
        c = roundf(c * 100) / 100;
        temperature = (c-273)*100;
    }
    return temperature;
}

void read_fan_pulses(collected_pulses_t pulse_set){
    int count_pulses = 0;
    int already_pulsed[NUM_FANS] = {0};
    HAL_TIM_Base_Start_IT(&htim14);
    while (!pulse_timout && count_pulses != NUM_FANS*2){

        switch (pulse_source)
        {
            case PUMP:
                if(!already_pulsed[PUMP]){
                    pulse_set[PUMP].p1 = TIM7->CNT;
                    already_pulsed[PUMP] = 1;
                    count_pulses++;
                }
                else{
                    pulse_set[PUMP].p2 = TIM7->CNT;
                    count_pulses++;
                }
            break;

            case FAN_1:
                if(!already_pulsed[FAN_1]){
                    pulse_set[FAN_1].p1 = TIM7->CNT;
                    already_pulsed[FAN_1] = 1;
                    count_pulses++;
                }
                else{
                    pulse_set[FAN_1].p2 =TIM7->CNT;
                    count_pulses++;
                }
            break;

            case FAN_2:
                if(!already_pulsed[FAN_2]){
                    pulse_set[FAN_2].p1 = TIM7->CNT;
                    already_pulsed[FAN_2] = 1;
                    count_pulses++;
                }
                else{
                    pulse_set[FAN_2].p2 = TIM7->CNT;
                    count_pulses++;
                }
            break;

            case FAN_3:
                if(!already_pulsed[FAN_3]){
                    pulse_set[FAN_3].p1 = TIM7->CNT;
                    already_pulsed[FAN_3] = 1;
                    count_pulses++;
                }
                else{
                    pulse_set[FAN_3].p2 = TIM7->CNT;
                    count_pulses++;
                }
            break;

            case FAN_4:
                if(!already_pulsed[FAN_4]){
                    pulse_set[FAN_4].p1 = TIM7->CNT;
                    already_pulsed[FAN_4] = 1;
                    count_pulses++;
                }
                else{
                    pulse_set[FAN_4].p2 = TIM7->CNT;
                    count_pulses++;
                }
            break;

            default:
            break;
        }
    }
    HAL_TIM_Base_Stop_IT(&htim14);

}

void init_pulses(collected_pulses_t pulse_set){
    for(int i = 0; i<NUM_FANS; i++){
        pulse_set[i].p1 = IMPOSSIBLE_VALUE_COUNTER;
        pulse_set[i].p2 = IMPOSSIBLE_VALUE_COUNTER;
    }
}


void fan_speed_calculator(uint32_t speeds[], collected_pulses_t  pulses){
	int elapsed_ticks = 0U;
	float elapsed_time = 0.0;
	float fan_freq = 0.0;
	int RPM = IMPOSSIBLE_VALUE_RPM;
	for(int i = 0; i<NUM_FANS; i++){
		elapsed_ticks = pulses[i].p2 - pulses[i].p1;
		elapsed_time = (float)elapsed_ticks/CLOCK_SPEED;
		fan_freq = 1/elapsed_time;
        fan_freq = roundf(fan_freq * 100) / 100;
		RPM = ((fan_freq*60)/2);
        if(RPM < MIN_SPEED_FAN || RPM > MAX_SPEED_FAN){
            speeds[i] = IMPOSSIBLE_VALUE_RPM;
        }
        else{
		    speeds[i] = RPM;
        }
	}
}



uint8_t process_buffer_command(char buff[], data_in_t *data_ui){
    char *token = strtok(buff, SEPARATOR);
    uint32_t hold_num[TEST_SIZE]; memset(hold_num, IMPOSSIBLE_VALUE_COUNTER, TEST_SIZE*sizeof(uint32_t));
    int i = 0;
        while (token != NULL){
            hold_num[i] =  atoi(token);
            i++;
            token = strtok(NULL, SEPARATOR);
        }
        if(hold_num[9] != IMPOSSIBLE_VALUE_COUNTER){
            data_ui->pwm_f0 = hold_num[0];
            data_ui->pwm_f1 = hold_num[1];
            data_ui->pwm_f2 = hold_num[2];
            data_ui->pwm_f3 = hold_num[3];
            data_ui->sw_1 = hold_num[4];
            data_ui->sw_2 = hold_num[5];
            data_ui->freq_pwm = hold_num[6];
            return 1;
        }
        else{
            return 0;
        }
}


uint8_t process_setup_data(char buff[], led_setup_t *data_setup){
    char *token = strtok(buff, SEPARATOR);
    uint32_t hold_num[TEST_SIZE]; memset(hold_num, IMPOSSIBLE_VALUE_COUNTER, sizeof(uint32_t)*TEST_SIZE);
    int i = 0;
    while (token != NULL){
        hold_num[i] =  atoi(token);
        i++;
        token = strtok(NULL, SEPARATOR);

    }
    if(hold_num[1]!= IMPOSSIBLE_VALUE_COUNTER){
        data_setup->num_led_fan = hold_num[0];
        data_setup->num_led_strip = hold_num[1];
        return 1;
    }
    else{
        return 0;
    }
}

uint32_t fragment_led(char *led_s){
    char *state;
    char *token = strtok_r(led_s,SEPARATOR, &state);
    uint32_t hold[3];
    int i = 0;
    while(token != NULL){
        hold[i] = atoi(token);
        i++;
        token = strtok_r(NULL, SEPARATOR,  &state);
    }
    return (uint32_t)( (hold[0]  << 16) | (hold[1] << 8) | (hold[2]));

}


uint8_t process_led_effect_data(char buff[], uint32_t effect[]){
    char *state;
    char *token_led = strtok_r(buff, SEPARATOR_LED, &state);
    char hold[TEST_SIZE] = {0};
    int i = 0;

    while (token_led != NULL){
        strcpy(hold, token_led);
        effect[i] = fragment_led(hold);
        i++;
        memset(hold, 0, TEST_SIZE);
        token_led = strtok_r(NULL, SEPARATOR_LED, &state);
    }
    return 1;
}

void set_speeds_in_data(uint32_t speeds[], data_out_t *data_out){
	data_out->rpm_pump = speeds[0];
	data_out->rpm0 = speeds[1];
	data_out->rpm1 = speeds[2];
	data_out->rpm2 = speeds[3];
	data_out->rpm3 = speeds[4];
}


void create_data_pcb(char* msg, data_out_t data){
 char hold[TEST_SIZE] = {0};
 sprintf(hold,"%lu", data.rpm0);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, ";");
 sprintf(hold,"%lu", data.rpm1);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, ";");
 sprintf(hold,"%lu", data.rpm2);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, ";");
 sprintf(hold,"%lu", data.rpm3);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, ";");
 sprintf(hold,"%lu", data.rpm_pump);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, ";");
 sprintf(hold,"%lu", data.temperature);
 strcat(msg, hold); memset(hold, 0, TEST_SIZE); strcat(msg, "#");
}


uint32_t create_data_ws(uint16_t *pwm_data, uint32_t effect[], led_setup_t leds){
	uint32_t index = 0;
	for(int j = 0; j<leds.num_led_fan; j++){
		for(int i = 23; i>=0; --i){
			if (effect[j]&(1<<i)){
				pwm_data[index] = LOGIC_ONE_LED;
			}
			else{
				pwm_data[index] = LOGIC_ZERO_LED;
			}
		}

	}
	for (int i=0; i<END_OF_EFFECT; i++){
		pwm_data[index] = 0;
		index++;
	}
	return index;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	data_in_t data_in;
	data_out_t data_out;
	led_setup_t led_setup;
	collected_pulses_t pulses;
	uint32_t speeds[NUM_FANS]; memset(speeds, IMPOSSIBLE_VALUE_RPM, NUM_FANS*sizeof(uint32_t));
	uint32_t *led_fan = NULL;
	uint32_t *led_strip = NULL;
	uint16_t *pwm_data = NULL;
	uint8_t current_fan = 0;
	char msg[TEST_SIZE] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_WWDG_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    switch (buffer[0]) {
    case CONFIG_DATA:
      if (!process_buffer_command((char * ) buffer, & data_in)) {
        break;
      }

      __HAL_TIM_SET_COMPARE( & htim3, TIM_CHANNEL_1, data_in.pwm_f0);
      if (data_in.pwm_f0 == 0) {
        HAL_GPIO_WritePin(SWITCH_FAN_1_GPIO_Port, SWITCH_FAN_1_Pin, RESET);
      } else {
        HAL_GPIO_WritePin(SWITCH_FAN_1_GPIO_Port, SWITCH_FAN_1_Pin, SET);
      }

      __HAL_TIM_SET_COMPARE( & htim3, TIM_CHANNEL_2, data_in.pwm_f1);
      if (data_in.pwm_f1 == 0) {
        HAL_GPIO_WritePin(SWITCH_FAN_2_GPIO_Port, SWITCH_FAN_2_Pin, RESET);
      } else {
        HAL_GPIO_WritePin(SWITCH_FAN_2_GPIO_Port, SWITCH_FAN_2_Pin, SET);
      }

      __HAL_TIM_SET_COMPARE( & htim3, TIM_CHANNEL_3, data_in.pwm_f2);
      if (data_in.pwm_f2 == 0) {
        HAL_GPIO_WritePin(SWITCH_FAN_3_GPIO_Port, SWITCH_FAN_3_Pin, RESET);
      } else {
        HAL_GPIO_WritePin(SWITCH_FAN_3_GPIO_Port, SWITCH_FAN_3_Pin, SET);
      }

      __HAL_TIM_SET_COMPARE( & htim3, TIM_CHANNEL_4, data_in.pwm_f3);
      if (data_in.pwm_f3 == 0) {
        HAL_GPIO_WritePin(SWITCH_FAN_4_GPIO_Port, SWITCH_FAN_4_Pin, RESET);
      } else {
        HAL_GPIO_WritePin(SWITCH_FAN_4_GPIO_Port, SWITCH_FAN_4_Pin, SET);
      }

      if (data_in.sw_1) {
        HAL_GPIO_WritePin(GEN_SWITCH_1_GPIO_Port, GEN_SWITCH_1_Pin, SET);
      } else {
        HAL_GPIO_WritePin(GEN_SWITCH_1_GPIO_Port, GEN_SWITCH_1_Pin, RESET);
      }

      if (data_in.sw_2) {
        HAL_GPIO_WritePin(GEN_SWITCH_2_GPIO_Port, GEN_SWITCH_2_Pin, SET);
      } else {
        HAL_GPIO_WritePin(GEN_SWITCH_2_GPIO_Port, GEN_SWITCH_2_Pin, RESET);
      }

      break;
    case SETUP_DATA:
      if (!process_setup_data((char * ) buffer, & led_setup)) {
        break;
      }
      if (led_strip != NULL) {
        free(led_strip);
      }
      if (led_fan != NULL) {
        free(led_fan);
      }
      if (pwm_data != NULL) {
        free(pwm_data);
      }
      led_fan = (uint32_t * ) calloc(led_setup.num_led_fan, sizeof(uint32_t) * led_setup.num_led_fan);
      led_strip = (uint32_t * ) calloc(led_setup.num_led_strip, sizeof(uint32_t) * led_setup.num_led_strip);
      pwm_data = (uint16_t * ) calloc(23 * led_setup.num_led_strip + END_OF_EFFECT, sizeof(uint32_t) * led_setup.num_led_strip);
      break;

    case LED_DATA:
      process_led_effect_data((char * ) buffer, led_fan);
      uint32_t total_size = create_data_ws(pwm_data, led_fan, led_setup);

      HAL_TIM_PWM_Start_DMA( & htim2, rgb_channels[current_fan], (uint32_t * ) pwm_data, total_size);
      current_fan++;
      if (current_fan == 4) {
        current_fan = 0;
      }
      while (!transfer_complete);
      transfer_complete = 0;

      break;

    default:
      HAL_Delay(NO_NOW_COMMAND_TIMEOUT);
      break;
    }

    data_out.temperature = convert_raw_to_temp(adc_read(hadc));
    init_pulses(pulses);
    read_fan_pulses(pulses);
    fan_speed_calculator(speeds, pulses);
    set_speeds_in_data(speeds, & data_out);

    HAL_GPIO_TogglePin(BOARD_STATE_GPIO_Port, BOARD_STATE_Pin);
    create_data_pcb(msg, data_out);
    CDC_Transmit_FS((uint8_t * ) msg, strlen(msg));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
