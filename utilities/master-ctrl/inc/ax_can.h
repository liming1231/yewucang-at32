#ifndef __AX_CAN_H__
#define __AX_CAN_H__
#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ax_common.h"

#define SET_F1_REBOOT_ID (0x201)
#define SET_F2_REBOOT_ID (0x211)
#define SET_F3_REBOOT_ID (0x221)
#define SET_F4_REBOOT_ID (0x231)

#define FB_F1_SET_REBOOT_ID (0x281)
#define FB_F2_SET_REBOOT_ID (0x291)
#define FB_F3_SET_REBOOT_ID (0x2A1)
#define FB_F4_SET_REBOOT_ID (0x2B1)

#define GET_F1_VERSION_ID (0x601)
#define GET_F2_VERSION_ID (0x611)
#define GET_F3_VERSION_ID (0x621)
#define GET_F4_VERSION_ID (0x631)

#define GET_HEARDBEAT_ID (0x600)

#define FB_F1_HEARDBEAT_ID (0x680)
#define FB_F2_HEARDBEAT_ID (0x690)
#define FB_F3_HEARDBEAT_ID (0x6A0)
#define FB_F4_HEARDBEAT_ID (0x6B0)

#define FB_GET_F1_VERSION_ID (0x681)
#define FB_GET_F2_VERSION_ID (0x691)
#define FB_GET_F3_VERSION_ID (0x6A1)
#define FB_GET_F4_VERSION_ID (0x6B1)

#define SET_F1_WS2812B_ID (0x701)
#define SET_F2_WS2812B_ID (0x711)
#define SET_F3_WS2812B_ID (0x721)
#define SET_F4_WS2812B_ID (0x731)

#define RESET_F1_SENSOR_ID (0x702)
#define RESET_F2_SENSOR_ID (0x712)
#define RESET_F3_SENSOR_ID (0x722)
#define RESET_F4_SENSOR_ID (0x732)

#define FB_SET_F1_WS2812B_ID (0x781)
#define FB_SET_F2_WS2812B_ID (0x791)
#define FB_SET_F3_WS2812B_ID (0x7A1)
#define FB_SET_F4_WS2812B_ID (0x7B1)

#define FB_RESET_F1_SENSOR_ID (0x782)
#define FB_RESET_F2_SENSOR_ID (0x792)
#define FB_RESET_F3_SENSOR_ID (0x7A2)
#define FB_RESET_F4_SENSOR_ID (0x7B2)

#define RECV_F1_DISTANCE_ID (0x7F1)
#define RECV_F2_DISTANCE_ID (0x7F2)
#define RECV_F3_DISTANCE_ID (0x7F3)
#define RECV_F4_DISTANCE_ID (0x7F4)

#define UNDEFINED (0x00)

extern can_rx_message_type rx_message_struct_g;
extern volatile uint8_t newMsgFlag;

/**
 * @brief       CANBUS开关管脚配置
 * @param[]     null
 * @return      null
 */
void canbus_sw_config(void);

/**
 * @brief       CANBUS通信打开
 * @param[]     null
 * @return      null
 */
void canbus_open(void);

/**
 * @brief       CANBUS通信状态结构体初始化
 * @param[null]
 * @return      null
 */
void can_alive_struct_init(void);

/**
 * @brief       CANBUS GPIO配置
 * @param[null]
 * @return      null
 */
void can_gpio_config(void);

/**
 * @brief       CANBUS通信配置
 * @param[null]
 * @return      null
 */
void can_configuration(void);

/**
 * @brief       CANBUS状态数据发送
 * @param[null]
 * @return      null
 */
void can_transmit_sts_data(void);

/**
 * @brief       发送CAN控制数据
 * @param[can_tx_message_type]
 * @return      null
 */
void can_transmit_ctrl_data(can_tx_message_type *tx_message_struct);

/**
 * @brief       δʹ��
 * @param[null]
 * @return      null
 */
void can_transmit_fbdata(void);

/**
 * @brief       CANBUS状态数据发送
 * @param[null]
 * @return      null
 */
void can_tx_task_function(void *pvParameters);

/**
 * @brief       CANBUS接收任务
 * @param[null]
 * @return      null
 */
void can_rx_task_function(void *pvParameters);

#endif
