#ifndef __AX_SPI__H__
#define __AX_SPI__H__

#include "at32f415_conf.h"
#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ax_common.h"

/**
 * @brief       SPI2 GPIO配置
 * @param[null]
 * @return      null
 */
void spi2_gpio_cfg(void);

/**
 * @brief       SPI2通信配置
 * @param[null]
 * @return      null
 */
void spi2_config(void);

/**
 * @brief       SPI2通信发送数据
 * @param[uint8_t*] 数据指针
 * @param[uint16_t] 数据长度
 * @return      null
 */
void spi2_send_bytes(uint8_t *datas, uint16_t len);

#endif
