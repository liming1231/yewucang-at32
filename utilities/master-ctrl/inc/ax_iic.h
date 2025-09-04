#ifndef __AX_IIC_H__
#define __AX_IIC_H__

#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c_application.h"
#include "stdbool.h"

#define I2C1_OWN_ADDRESS (0x50)

// #define I2C_TIMEOUT     0xFFFFFFFF
#define I2C_TIMEOUT 0x000FFFFF
#define I2C1_SPEED (100000)
// #define I2C1_ADDRESS    (0x52)
#define I2C1_PORT I2C1
#define I2C2_PORT I2C2
#define I2C1_CLK CRM_I2C1_PERIPH_CLOCK

#define I2C1_SCL_PIN GPIO_PINS_6
#define I2C1_SCL_GPIO_PORT GPIOB
#define I2C1_SCL_GPIO_CLK CRM_GPIOB_PERIPH_CLOCK
#define I2C1_SDA_PIN GPIO_PINS_7
#define I2C1_SDA_GPIO_PORT GPIOB
#define I2C1_SDA_GPIO_CLK CRM_GPIOB_PERIPH_CLOCK

#define I2C1_BUF_SIZE (32)
#define I2C2_BUF_SIZE (16)

#define I2C1_XSHUT_1_PIN GPIO_PINS_3
#define I2C1_XSHUT_1_PORT GPIOC
#define I2C1_XSHUT_2_PIN GPIO_PINS_4
#define I2C1_XSHUT_2_PORT GPIOC
#define I2C1_XSHUT_3_PIN GPIO_PINS_5
#define I2C1_XSHUT_3_PORT GPIOC

extern uint8_t I2C1_ADDRESS[3];

enum vcselPeriodType
{
    VcselPeriodPreRange,
    VcselPeriodFinalRange
};

extern uint8_t hi2c1_tx_buf[I2C1_BUF_SIZE];
extern uint8_t hi2c1_rx_buf[I2C1_BUF_SIZE];
extern uint8_t hi2c2_tx_buf[I2C2_BUF_SIZE];
extern uint8_t hi2c2_rx_buf[I2C2_BUF_SIZE];
extern i2c_handle_type hi2c1, hi2c2;
extern i2c_status_type hi2c1_i2c_status;
extern i2c_status_type hi2c2_i2c_status;

extern void i2c_xshut_init(void);

void error_handler(uint32_t error_code);
uint32_t buffer_compare(uint8_t *buffer1, uint8_t *buffer2, uint32_t len);
void i2c_lowlevel_init(i2c_handle_type *hi2c);
void i2c1_task_function(void *pvParameters);

#endif
