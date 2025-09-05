#ifndef __AX_COMMON__H__
#define __AX_COMMON__H__

#include "at32f415_conf.h"
#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define VERSION (0x31303034)

// #define DEBUG

#define RUN_LED_CLK CRM_GPIOC_PERIPH_CLOCK
#define RUN_LED_PORT GPIOC
#define RUN_LED_PIN GPIO_PINS_13
#define USBHUB_RST_CLK CRM_GPIOC_PERIPH_CLOCK
#define USBHUB_RST_PORT GPIOC
#define USBHUB_RST_PIN GPIO_PINS_5
#define SWITCH_PWR_CLK CRM_GPIOA_PERIPH_CLOCK
#define SWITCH_PWR_PORT GPIOA
#define SWITCH_PWR_PIN GPIO_PINS_6
#define ANDRIOD_PWR_CLK CRM_GPIOA_PERIPH_CLOCK
#define ANDRIOD_PWR_PORT GPIOA
#define ANDRIOD_PWR_PIN GPIO_PINS_7
#define CAR_CMPUTER_ACC_CLK CRM_GPIOB_PERIPH_CLOCK
#define CAR_CMPUTER_ACC_PORT GPIOB
#define CAR_CMPUTER_ACC_PIN GPIO_PINS_7

/* CANBUS开关管脚配置 */
#define CAN_BUS_SHON_CLOCK CRM_GPIOA_PERIPH_CLOCK
#define CAN_BUS_SHON_PORT GPIOA
#define CAN_BUS_SHON_PIN GPIO_PINS_11
#define CAN_BUS_RS_CLOCK CRM_GPIOA_PERIPH_CLOCK
#define CAN_BUS_RS_PORT GPIOA
#define CAN_BUS_RS_PIN GPIO_PINS_12

/* uart 发送 */
#define UART1_GPIO_CLOCK CRM_GPIOA_PERIPH_CLOCK
#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN GPIO_PINS_9
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN GPIO_PINS_10
#define UART2_GPIO_CLOCK CRM_GPIOA_PERIPH_CLOCK
#define UART2_TX_PORT GPIOA
#define UART2_TX_PIN GPIO_PINS_2
#define UART2_RX_PORT GPIOA
#define UART2_RX_PIN GPIO_PINS_3
#define UART3_GPIO_CLOCK CRM_GPIOB_PERIPH_CLOCK
#define UART3_TX_PORT GPIOB
#define UART3_TX_PIN GPIO_PINS_10
#define UART3_RX_PORT GPIOB
#define UART3_RX_PIN GPIO_PINS_11

/* spi 发送 */
#define SPI2_MOSI_PORT GPIOB
#define SPI2_MOSI_PIN GPIO_PINS_15

/* 任务状态 */
#define TASK_DEBUG_LED_BIT_1 (0x01 << 0)
#define TASK_UART_RX_BIT_2 (0x01 << 1)
#define TASK_UART_TX_BIT_3 (0x01 << 2)
#define TASK_CAN_TX_BIT_4 (0x01 << 3)
#define TASK_CAN_RX_BIT_5 (0x01 << 4)
#define TASK_BIT_ALL (TASK_DEBUG_LED_BIT_1 | TASK_UART_RX_BIT_2 | TASK_UART_TX_BIT_3 | TASK_CAN_TX_BIT_4 | TASK_CAN_RX_BIT_5)

enum DATA_VALIDITY
{
    INVALID = 0x00,
    VALID = 0x01
};
enum CTRL_ACTION
{
    TURN_OFF = 0x00,
    TURN_ON = 0x01
};

struct uart_send_flag
{
    uint8_t version_own;
    uint8_t version_f1;
    uint8_t version_f2;
    uint8_t version_f3;
    uint8_t version_f4;
    uint8_t ctrl_leds;
    uint8_t reset_sensor;
    uint8_t need_reboot;
    uint8_t reset_usbhub;
    uint8_t reset_switch;
    uint8_t reset_andriod;
    uint8_t reset_acc;
    uint8_t uid_own;
    uint8_t uid_f1;
    uint8_t uid_f2;
    uint8_t uid_f3;
    uint8_t uid_f4;
};

struct can_alive_counter
{
    uint8_t canNowCounter[4];
    uint8_t canLastCounter[4];
    uint8_t canTimeoutCounter[4];
    uint8_t onLine[4];
    uint8_t loopCounter;
};

extern EventBits_t taskAliveBits;
extern volatile uint16_t distance[4][6];
extern struct uart_send_flag uart1SendTypeFlag;
extern struct can_alive_counter canAliveCounter;
extern uint8_t versionSub[4][6];
extern uint8_t uidBuf[5][12];

/**
 * @brief       计算CRC16校验
 * @param[uint8_t*] 数据指针
 * @param[uint8_t]  数据长度
 * @return      null
 */
uint16_t crc16_modbus(uint8_t *pszBuf, uint8_t unLength);

#endif
