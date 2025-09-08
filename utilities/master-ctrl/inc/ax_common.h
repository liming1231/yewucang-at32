#ifndef __AX_COMMON__H__
#define __AX_COMMON__H__

#include "at32f415_conf.h"
#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

// #define VERSION (0x313136)

// #define DEBUG
#define CAN_DAUL
#ifdef CAN_DAUL
#define VERSION (0x31323231)
#else
#define VERSION (0x31323230)
#endif
#define IHAWK_CTRL

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

#define IHAWK_POWER_1_CLK CRM_GPIOB_PERIPH_CLOCK
#define IHAWK_POWER_1_PORT GPIOB
#define IHAWK_POWER_1_PIN GPIO_PINS_5
#define IHAWK_POWER_2_CLK CRM_GPIOB_PERIPH_CLOCK
#define IHAWK_POWER_2_PORT GPIOB
#define IHAWK_POWER_2_PIN GPIO_PINS_6

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
#define TASK_UART_RX_BIT_6 (0x01 << 5)
#define TASK_BIT_ALL (TASK_DEBUG_LED_BIT_1 | TASK_UART_RX_BIT_2 | TASK_UART_TX_BIT_3 | TASK_CAN_TX_BIT_4 | TASK_CAN_RX_BIT_5 | TASK_UART_RX_BIT_6)

#define ROM_BASE_ADDR (0x8000000)
#define START_APP_FLAG_ADDR (0x8003800)
#define RUNNING_APP1_ADDR (0x8004000)
#define RUNNING_APP2_ADDR (0x801C000)
#define APP_FW_MAX_LEN (0x18000)

#define APP1_FLAG (0x01)
#define APP2_FLAG (0x02)

#define MAX_BRIGHTNESS (0x80)

enum UPDATE_FW_STEP
{
    UPDATE_NONE = 0,
    UPDATE_FW_INFO = 1,
    UPDATE_FW_START = 2,
    UPDATE_FW_ING = 3,
    UPDATE_END = 4
};

enum DATA_VALIDITY
{
    INVALID = 0x00,
    VALID = 0x01
};
enum CTRL_ACTION
{
    TURN_OFF = 0x00,
    TURN_ON = 0x01,
    RESET_IHAWK = 0x02
};
enum CTRL_USB_CS
{
    LOWER_USB = 0x01,
    UPPER_USB = 0x02,
    DUAL_USB = 0x03
};

enum INTERGRATD_GROUP_TYPE
{
    GROUP_NULL = 0x00,
    GROUP_REBOOT = 0x01,
    GROUP_CTRL_USB = 0x02,
    GROUP_CTRL_ANDRIOD = 0x03,
    GROUP_CTRL_ACC = 0x04
};

// ��Դ����ָ��
enum PowerActionType
{
    ACTION_OFF = 0,  // ��
    ACTION_ON = 1,   // ��
    ACTION_RESET = 2 // ���ã��ȹ��ٿ���
};

typedef struct _dev_index
{
    uint16_t devList;
    uint16_t ind_1 : 1;  // bit 0
    uint16_t ind_2 : 1;  // bit 1
    uint16_t ind_3 : 1;  // bit 2
    uint16_t ind_4 : 1;  // bit 3
    uint16_t ind_5 : 1;  // bit 4
    uint16_t ind_6 : 1;  // bit 5
    uint16_t ind_7 : 1;  // bit 6
    uint16_t ind_8 : 1;  // bit 7
    uint16_t ind_9 : 1;  // bit 8
    uint16_t ind_10 : 1; // bit 9
    uint16_t ind_11 : 1; // bit 10
    uint16_t ind_12 : 1; // bit 11
    uint16_t ind_13 : 1; // bit 12
    uint16_t ind_14 : 1; // bit 13
    uint16_t ind_15 : 1; // bit 14
    uint16_t ind_16 : 1; // bit 15

} dev_index;

#pragma pack(1)
typedef struct _integrated_cmd_struct
{
    uint8_t group;
    dev_index devIndex;
    uint16_t cmdIndex;
    uint8_t actionType;
    uint8_t holdTm;
    uint8_t fbIntegratedCmdBuffer[128];
    uint8_t fbIntegratedCmdBufferLen;
} integrated_cmd_struct;

typedef struct _can_fw_info
{
    uint8_t sync_sub_fw;
    uint8_t sync_sub_fw_valid;
    uint8_t update_addr;
    uint16_t fw_length;
    uint16_t fw_msg_counter;
    uint16_t fw_index;

} can_fw_info;

struct uart_send_flag
{
    uint8_t version_own;
    uint8_t version_own_valid;
    uint8_t version_f1;
    uint8_t version_f2;
    uint8_t version_f3;
    uint8_t version_f4;
    uint8_t ctrl_diy_leds_fb;
    uint8_t ctrl_diy_leds_index;
    uint8_t ctrl_diy_leds_valid;

    uint8_t ctrl_diy2_leds_fb;
    uint8_t ctrl_diy2_leds_index;
    uint8_t ctrl_diy2_leds_valid;

    uint8_t ctrl_leds_fb;
    uint8_t ctrl_leds_index;
    uint8_t ctrl_leds_valid;

    uint8_t ctrl_tray_leds_fb;
    uint8_t ctrl_tray_leds_index;
    uint8_t ctrl_tray_leds_valid;

    uint8_t reset_sensor;
    uint8_t reset_sensor_valid;
    uint8_t need_reboot_fb;
    uint8_t need_reboot_index;
    uint8_t need_reboot_valid;
    uint8_t reset_usbhub;
    uint8_t reset_usbhub_valid;
    uint8_t reset_switch;
    uint8_t reset_switch_valid;
    uint8_t reset_andriod;
    uint8_t reset_andriod_valid;
    uint8_t reset_acc;
    uint8_t reset_acc_valid;
    can_fw_info fw_info;
    uint8_t uid_own;
    uint8_t uid_own_valid;
    uint8_t uid_f1;
    uint8_t uid_f2;
    uint8_t uid_f3;
    uint8_t uid_f4;

    uint8_t actionDir;
    uint8_t set_roller_valid;
    uint8_t set_roller;
    uint16_t roll_spd;
};

struct can_alive_counter
{
    uint8_t canNowCounter[4];
    uint8_t canLastCounter[4];
    uint8_t canTimeoutCounter[4];
    uint8_t onLine[4];
    uint8_t loopCounter;
};

struct _send_can_str
{
    uint8_t setWs2812b[4];
    uint8_t resetSensor[4];
    uint8_t resetBoard[4];
    uint8_t sync_fw[4];
    uint8_t setTrayWs2812b;
};

struct _ihawk_power_sts
{
    uint8_t ihawk_sts_1;
    uint8_t ihawk_sts_2;
};
#pragma pack()

extern EventBits_t taskAliveBits;
extern volatile uint16_t distance[4][6];
extern struct uart_send_flag uart1SendTypeFlag;
extern struct uart_send_flag uart2SendTypeFlag;
extern struct can_alive_counter canAliveCounter;
extern struct _send_can_str sendCanStr;

extern struct _ihawk_power_sts ihawk_power_sts;

extern integrated_cmd_struct integratedCmdStruct;

extern can_tx_message_type canResetSensor[4];
extern can_tx_message_type canSetLeds[4];
extern can_tx_message_type canSetTrayLeds;
extern can_tx_message_type canResetBoard[4];
extern can_tx_message_type canSyncFw[4];

extern uint8_t versionSub[4][6];
extern uint8_t uidBuf[5][12];

extern uint8_t updateFwData[16];
extern volatile uint8_t msglen2;
extern volatile uint8_t updateFWFlag;
extern volatile uint8_t xorData[2];

/**
 * @brief       计算CRC16校验
 * @param[uint8_t*] 数据指针
 * @param[uint8_t]  数据长度
 * @return      null
 */
uint16_t crc16_modbus(uint8_t *pszBuf, uint8_t unLength);

uint8_t crc8_rcc(uint8_t a, uint8_t b);

#endif
