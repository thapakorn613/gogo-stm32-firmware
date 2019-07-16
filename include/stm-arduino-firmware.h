#ifndef __STM_ARDUINO_FIRMWARE_H__
#define __STM_ARDUINO_FIRMWARE_H__

//* ===================================================
//*  Commands Definitions
//* ===================================================

#define LED_PIN PC13 

//* ===================================================
//*  Commands Definitions
//* ===================================================
#define CMD_PACKET                        0
#define FLASH_MEMORY_OPERATION_PACKET     1
#define DEVICE_REGISTER_OPERATION_PACKET  2
#define DECENTRALIZED_OPERATION           5

#define CMD_PING                          1

#define CMD_CATEGORY_ID                   1
#define CMD_READ_REG                      1
#define CMD_SEND_STM_RX                   2

#define CMD_MOTOR_ON_OFF                  2
#define CMD_MOTOR_DIRECTION               3
#define CMD_MOTOR_RD                      4
#define CMD_SET_POWER                     6
#define CMD_SET_ACTIVE_PORTS              7
#define CMD_TOGGLE_ACTIVE_PORT            8
#define CMD_SET_SERVO_DUTY                9
#define CMD_LED_CONTROL                   10
#define CMD_BEEP                          11
#define CMD_AUTORUN_STATE                 12
#define CMD_LOGO_CONTROL                  13

#define CMD_REBOOT                        100

//* ===================================================
//*  Report Register Names
//* ===================================================
#define REPORT_PACKET_TYPE                       0
// #define REPORT_MOTOR_ACTIVE_PORT                 1
// #define REPORT_MOTOR_STATUS                      2
// #define REPORT_MOTOR_DIR                         3
// #define REPORT_MOTOR_PWR                         4
// #define REPORT_IR_CODE_RECEIVED                  8
#define REPORT_FIRMWARE_ID                       15

//* ===================================================
//*  STM PERIPHERALS
//* ===================================================


#endif