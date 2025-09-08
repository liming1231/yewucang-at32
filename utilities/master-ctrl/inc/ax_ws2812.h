#ifndef __AX_WS2812_H__
#define __AX_WS2812_H__

#include "ax_spi.h"

// 每个LED的字节数
#define ONE_LED_TYPES (24)
// LED数量
#define LED_SUM (56)
// 一个LED数组的字节数
#define LED_ARR (ONE_LED_TYPES * LED_SUM) // 60*24

#define WS2812_BIT_0 (0xC0)
#define WS2812_BIT_1 (0xFC)

#define LED_COLOR_TYPE_MAX 0x07
enum LED_COLOR
{
    OFF_COLOR = 0,
    RED_COLOR = 0x01,
    GREEN_COLOR = 0x02,
    BLUE_COLOR = 0x03,
    YELLOW_COLOR = 0x04,
    CYAN_COLOR = 0x05,
    MAGENTA_COLOR = 0x06,
    WHITE_COLOR = 0x07
};

#define LED_MODE_MAX 0x09
enum LED_MODE
{
    OFF_MODE = 0,
    NORMAL_MODE = 0x01,
    FLASH_MODE = 0x02,
    BREATH_2S = 0x03,
    BREATH_4S = 0x04,
    DIY_SHOW = 0x05,
    DIY2_SHOW = 0x06,
    RAINBOW = 0x07,
    FLASH_1HZ_MODE = 0x08,
    FLASH_2HZ_MODE = 0x09
};

typedef struct
{
    uint8_t g;
    uint8_t r;
    uint8_t b;
} COLOR;

extern COLOR color_grb;
extern uint8_t ledMode;
extern uint8_t ledHeaderCount;
extern uint8_t u8TxData[LED_ARR];

extern uint8_t diyArr[4][2];
extern uint8_t diyColor;

extern volatile uint8_t rainbow_percent;

/**
 * @brief      RGB LED显示初始化
 * @param[null]
 * @return      null
 */
void initColor(void);

/**
 * @brief       WS2812 LED驱动初始化
 * @param[null]
 * @return      null
 */
void ws2812Init(void);

/**
 * @brief       设置指定颜色模式
 * @param[COLOR]
 * @return      null
 */
void setNormalMode(COLOR color);

/**
 * @brief       设置红色模式
 * @param[null]
 * @return      null
 */
void setNormalRedMode(void);

/**
 * @brief       �رյƴ�
 * @param[null]
 * @return      null
 */
void setOffMode(void);

/**
 * @brief       设置呼吸灯模式
 * @param[COLOR]
 * @return      null
 */
void setBreath_4s(COLOR color);

/**
 * @brief       设置DIY显示模式
 * @param[COLOR]
 * @return      null
 */
void setDiyShow(COLOR color);

/**
 * @brief       设置呼吸灯模式
 * @param[COLOR]
 * @return      null
 */
void setBreath_2s(COLOR color);

/**
 * @brief       设置双闪烁模式
 * @param[COLOR]
 * @return      null
 */
void setDoubleFlashMode(void);

/**
 * @brief       设置指定颜色1Hz闪烁显示模式
 * @param[COLOR]
 * @return      null
 */
void setDoubleFlashRed_1Hz_Mode(void);

void setDoubleFlash_1Hz_Mode(COLOR color);

/**
 * @brief       设置指定颜色2Hz闪烁显示模式
 * @param[COLOR]
 * @return      null
 */
void setDoubleFlashRed_2Hz_Mode(void);

void setDoubleFlash_2Hz_Mode(COLOR color);

/**
 * @brief       设置分段显示模式
 * @param[COLOR]    前半段颜色
 * @param[COLOR]    后半段颜色
 * @param[uint8_t] 段落长度
 * @return      null
 */
void setSegmentedMode(COLOR color, COLOR color2, uint8_t sum_header);

/**
 * @brief           LED更新显示
 * @param[null]
 * @return      null
 */
void ledUpdate(void);

/**
 * @brief       彩虹循环显示
 * @param[]     null
 * @return      null
 */
void rainbowCycle(void);

/**
 * @brief       WS2812 LED驱动任务
 * @param[]     null
 * @return      null
 */
void ws2812_task_function(void *pvParameters);

#endif
