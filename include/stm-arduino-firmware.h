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

#define STATUS_LED PC13

#define CMD_PING                          1

#define CMD_CATEGORY_ID                   1
#define CMD_READ_REG                      1
#define CMD_SEND_STM_RX                   2

#define CMD_REBOOT                        100

//* ===================================================
//*  Report Register Names
//* ===================================================
#define REPORT_PACKET_TYPE                       0
#define REPORT_FIRMWARE_ID                       15

//* ===================================================
//*  STM PERIPHERALS
//* ===================================================


#endif