#ifndef __AX_UART_H__
#define __AX_UART_H__

#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ax_ws2812.h"
#include "ax_common.h"

#define UART_BAUDRATE_115200 (115200)

#define COUNTOF(a) (sizeof(a) / sizeof(*(a)))
#define USART_1_2_TX_BUFFER_SIZE (128u)
#define USART_1_2_RX_BUFFER_SIZE (256u)

#define GET_UPDATE_APP_FLAG_ID (0x88)
#define SET_UPDATE_FW_INFO_ID (0x89)
#define SET_UPDATE_START_ID (0x8A)
#define SET_UPDATE_DATA_ID (0x8B)
#define SET_UPDATE_END_ID (0x8C)
#define SET_UPDATE_ABORT_ID (0x8D)

#define FB_GET_UPDATE_APP_FLAG_ID (0x98)
#define FB_SET_UPDATE_FW_INFO_ID (0x99)
#define FB_SET_UPDATE_START_ID (0x9A)
#define FB_SET_UPDATE_DATA_ID (0x9B)
#define FB_SET_UPDATE_END_ID (0x9C)
#define FB_SET_UPDATE_ABORT_ID (0x9D)

#define REBOOT_CMD_ID (0x101)
#define GET_VERSION_CMD_ID (0x102)
// #define SET_LEDS_CMD_ID         (0x103)
// #define RESET_SENSOR_CMD_ID     (0x104)
#define RESET_USBHUB_CMD_ID (0x105)
#define RESET_SWITCH_CMD_ID (0x106)
#define RESET_ANDRIOD_CMD_ID (0x107)
#define RESET_ACC_CMD_ID (0x108)
// #define GET_UID_CMD_ID          (0x109)
// #define SET_TRAY_LEDS_CMD_ID    (0x10A)

#define SET_IHAWK_CMD_ID (0x10B)
#define GET_IHAWK_STS_ID (0x10C)

// #define SYNC_SUB_FW_INFO_ID     (0x10D)
// #define SUB_UPDATING_APP_ID     (0x10F)
// #define SET_DIY_CMD_ID          (0x110)
// #define SET_DIY2_CMD_ID         (0x111)

#define FB_REBOOT_CMD_ID (0x201)
#define FB_GET_VERSION_CMD_ID (0x202)
// #define FB_SET_LEDS_CMD_ID      (0x203)
// #define FB_RESET_SENSOR_CMD_ID  (0x204)
#define FB_RESET_USBHUB_CMD_ID (0x205)
#define FB_RESET_SWITCH_CMD_ID (0x206)
#define FB_RESET_ANDRIOD_CMD_ID (0x207)
#define FB_RESET_ACC_CMD_ID (0x208)
#define FB_GET_UID_CMD_ID (0x209)
#define FB_SET_TRAY_LEDS_CMD_ID (0x20A)

#define FB_SET_IHAWK_CMD_ID (0x20B)
#define FB_GET_IHAWK_STS_ID (0x20C)

// #define FB_SYNC_SUB_FW_INFO_ID  (0x20D)
// #define FB_SUB_UPDATING_APP_ID  (0x20F)
#define FB_SET_DIY_CMD_ID (0x210)
#define FB_SET_DIY2_CMD_ID (0x211)

#define TRAY_MASTER (0x00)
#define TRAY_F1 (0x01)
#define TRAY_F2 (0x02)
#define TRAY_F3 (0x03)
#define TRAY_F4 (0x04)
#define TRAY_SUM (0x04)

#define VERSION_INDEX_F1 (0x00)
#define VERSION_INDEX_F2 (0x01)
#define VERSION_INDEX_F3 (0x02)
#define VERSION_INDEX_F4 (0x03)

#define UART_MSG_HEADER_1 (0x55)
#define UART_MSG_HEADER_2 (0xAA)

#define UART_DATA_INIT (0x00)
#define UART_GET_HEADER_1 (0x01)
#define UART_GET_HEADER_2 (0x02)
#define UART_GET_MSG_DATA_LEN (0x03)
#define UART_MSG_DATA_LEN_MAX (128 + 5)

#define CTRL_OWNER_FLAG (0x0F)

#define AT32_CTRL_USBHUB_CMD_ID (0x0006)
#define AT32_CTRL_WS2812B_CMD_ID (0x0011)
#define AT32_CTRL_ACC_CMD_ID (0x0021)
#define AT32_GET_VERSION_CMD_ID (0x0022)
#define AT32_CTRL_GATE_23_CMD_ID (0x0023)
#define AT32_CTRL_GATE_24_CMD_ID (0x0024)
#define AT32_RESET_SWITCH_CMD_ID (0x0025)
#define AT32_RESET_ANDRIOD_CMD_ID (0x0026)
#define AT32_REBOOT_CMD_ID (0x0027)

struct uart_data
{
    uint8_t usart_tx_buffer[USART_1_2_TX_BUFFER_SIZE];
    uint8_t usart_rx_buffer[USART_1_2_RX_BUFFER_SIZE];
    uint8_t usart_tx_counter;
    uint8_t usart_rx_counter;
    uint8_t usart_tx_buffer_size;
    uint8_t usart_rx_buffer_size;
};

extern struct uart_data uart1_data, uart2_data;

extern uint8_t ctrl_buff[32];
extern uint8_t ctrl_buff2[256];
extern uint8_t update_buff[16][128];

extern TimerHandle_t xTimer;
extern uint32_t timerCnt;

void vTimerCallback(xTimerHandle pxTimer);

/**
 * @brief       设备控制引脚初始化
 * @param[null]
 * @return      null
 */
void device_ctrl_pins_init(void);

/**
 * @brief       USBHUB控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void usbhub_ctrl(uint8_t sts);

/**
 * @brief       开关电源控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void switch_pwr_ctrl(uint8_t sts);

/**
 * @brief       安卓电源控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void andriod_pwr_ctrl(uint8_t sts);

/**
 * @brief       汽车ACC电源控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void car_acc_ctrl(uint8_t sts);

/**
 * @brief       ihawk1电源控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void ihawk_lower_ctrl(uint8_t sts);

/**
 * @brief       ihawk2电源控制接口
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void ihawk_upper_ctrl(uint8_t sts);

/**
 * @brief       ihawk电源控制接口
 * @param[index]  选择1/2
 * @param[uint8_t]  状态，0-关闭，1-开启
 * @return      null
 */
void ihawk_ctrl(uint8_t index, uint8_t sts);

/**
 * @brief       打开设备电源
 * @param[null]
 * @return      null
 */
void open_device_pwr(void);

/**
 * @brief       切换指示灯状态
 * @param[null]
 * @return      null
 */
void toggle_led_stat(void);

/**
 * @brief       uart发送任务
 * @param[null]
 * @return      null
 */
void usart1_tx_task_function(void *pvParameters);

/**
 * @brief       uart接收任务
 * @param[null]
 * @return      null
 */
void usart1_rx_task_function(void *pvParameters);

void usart2_rx_task_function(void *pvParameters);

/**
 * @brief       uart配置
 * @param[null]
 * @return      null
 */
void usart_configuration(void);

/**
 * @brief       uart中断配置
 * @param[null]
 * @return      null
 */
void usart_int_configuration(void);

/**
 * @brief       uart2数据初始化
 * @param[null]
 * @return      null
 */
void init_uart_data(void);

#endif
