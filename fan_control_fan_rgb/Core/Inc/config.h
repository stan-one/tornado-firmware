#ifndef CONFIG_H
#define CONFIG_H

//TEMPERATURE
#define V_SUPPLY 3.3F
#define ADC_RES 4096U
#define MAX_VOLT 3.3F
#define BETA_TERMISTOR 3380U
#define RES_AT_25C 10000U
#define R1 10000U
//FANS
#define CLOCK_SPEED 48000U //using a prescales(99), down from 48MHz
#define NUM_FANS 5U
#define MAX_SPEED_FAN 2500
#define MIN_SPEED_FAN 300

#define IMPOSSIBLE_VALUE_RPM 1U
#define IMPOSSIBLE_VALUE_COUNTER 99999U// BIGGER THAN 16 BIT
#define IMPOSSIBLE_VALUE_TEMP 200U

//INPUT PROCESSING
#define LED_DATA '$'
#define CONFIG_DATA '@'
#define SETUP_DATA '&'
#define USB_BUFFER_SIZE 256
#define UI_COMMAND_LEN 35
#define END_CHAR '#'
#define SEPARATOR ";"
#define SEPARATOR_LED "|"

#define LOGIC_ONE_LED 40
#define LOGIC_ZERO_LED 20

#define END_OF_EFFECT 27 //NOT SURE ABOUT THIS
#define NO_NOW_COMMAND_TIMEOUT 100

enum pulse_source{TACH_PUMP, TACH_FAN_1, TACH_FAN_2, TACH_FAN_3, TACH_FAN_4};

#endif
