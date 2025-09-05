#include "ax_ws2812.h"
#include "stdbool.h"
#include "ax_spi.h"
#include "string.h"

COLOR color_grb = {0};
/// sin(x*pi/50)
const float sinLed[] = {0.000, 0.063, 0.125, 0.187, 0.249, 0.309, 0.368, 0.426, 0.482, 0.536, 0.588, 0.637, 0.685, 0.729, 0.771, 0.809, 0.844, 0.876, 0.905, 0.930, 0.951, 0.969, 0.982, 0.992, 0.998, 1.000, 0.998, 0.992, 0.982, 0.969, 0.951, 0.930, 0.905, 0.876, 0.844, 0.809, 0.771, 0.729, 0.685, 0.637, 0.588, 0.536, 0.482, 0.426, 0.368, 0.309, 0.249, 0.187, 0.125, 0.063};
/// sin(x*pi/100)
const float sinLed_4s[] = {0.000, 0.031, 0.063, 0.094, 0.125, 0.156, 0.187, 0.218, 0.249, 0.279, 0.309, 0.339, 0.368, 0.397, 0.426, 0.454, 0.482, 0.509, 0.536, 0.562, 0.588, 0.613, 0.637, 0.661, 0.685, 0.707, 0.729, 0.750, 0.771, 0.790, 0.809, 0.827, 0.844, 0.861, 0.876, 0.891, 0.905, 0.918, 0.930, 0.941, 0.951, 0.960, 0.969, 0.976, 0.982, 0.988, 0.992, 0.996, 0.998, 1.000, 1.000, 1.000, 0.998, 0.996, 0.992, 0.988, 0.982, 0.976, 0.969, 0.960, 0.951, 0.941, 0.930, 0.918, 0.905, 0.891, 0.876, 0.861, 0.844, 0.827, 0.809, 0.790, 0.771, 0.750, 0.729, 0.707, 0.685, 0.661, 0.637, 0.613, 0.588, 0.562, 0.536, 0.509, 0.482, 0.454, 0.426, 0.397, 0.368, 0.339, 0.309, 0.279, 0.249, 0.218, 0.187, 0.156, 0.125, 0.094, 0.063, 0.031};

static uint8_t led_sin_4s = 0;
static uint8_t led_sin_2s = 0;

static u8 pixelBuffer[LED_SUM][24]; // 根据灯珠数量定义数组大小

uint8_t diyArr[4][2] = {0x00};
uint8_t diyColor = 0;

uint8_t ledMode = 0;
uint8_t ledHeaderCount = LED_SUM;

volatile uint8_t rainbow_percent = 0;

uint8_t u8TxData[LED_ARR] =
    {
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,

        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,

        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,

        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,

        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,

        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1};

const uint8_t OFF_UNIT[24] =
    {
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0};

const uint8_t RED_UNIT[24] =
    {
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0};

const uint8_t GREEN_UNIT[24] =
    {
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0};
const uint8_t BLUE_UNIT[24] =
    {
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1};
const uint8_t YELLOW_UNIT[24] =
    {
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0};
const uint8_t CYAN_UNIT[24] =
    {
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1};
const uint8_t MAGENTA_UNIT[24] =
    {
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
        WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1};
const uint8_t WHITE_UNIT[24] =
    {
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_1,
        WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_0, WS2812_BIT_1};

const uint8_t DIY_UINT[8][24] =
    {
        {
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
        },
        {
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
        },
        {
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
        },

        {
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
        },
        {
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
        },
        {
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
        },
        {
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_0,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
            WS2812_BIT_1,
        },
        {WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
         WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1,
         WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1, WS2812_BIT_1}};

void initColor(void)
{
    color_grb.g = 0x00;
    color_grb.r = 0x00;
    color_grb.b = MAX_BRIGHTNESS;

    ledMode = BREATH_2S;
}

void ws2812Init(void)
{
    spi2_gpio_cfg();
    spi2_config();
    initColor();
}

void sendLED(COLOR color, uint8_t index)
{
    //  rewrite a led
    int i;

    for (i = 7; i >= 0; i--)
    {
        u8TxData[i + index * ONE_LED_TYPES] = ((color.g >> (7 - i)) & 0x01) ? WS2812_BIT_1 : WS2812_BIT_0;
        u8TxData[i + 8 + index * ONE_LED_TYPES] = ((color.r >> (7 - i)) & 0x01) ? WS2812_BIT_1 : WS2812_BIT_0;
        u8TxData[i + 16 + index * ONE_LED_TYPES] = ((color.b >> (7 - i)) & 0x01) ? WS2812_BIT_1 : WS2812_BIT_0;
    }
}

void setNormalMode(COLOR color)
{
    int i;
    sendLED(color, 0);

    for (i = 1; i < LED_SUM; i++)
    {
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);
    }

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);
}

void setNormalRedMode(void)
{
    int i;
    COLOR color_grb_t;
    color_grb_t.g = 0;
    color_grb_t.r = MAX_BRIGHTNESS;
    color_grb_t.b = 0;
    sendLED(color_grb_t, 0);

    for (i = 1; i < LED_SUM; i++)
    {
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);
    }

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);
}

void setOffMode(void)
{
    int i;
    COLOR color_grb_t;
    color_grb_t.g = 0;
    color_grb_t.r = 0;
    color_grb_t.b = 0;
    sendLED(color_grb_t, 0);

    for (i = 1; i < LED_SUM; i++)
    {
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);
    }

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);
}

void setBreath_4s(COLOR color)
{
    int i;
    COLOR color_grb_t;
    color_grb_t.g = (uint8_t)(color.g * sinLed_4s[led_sin_4s]);
    color_grb_t.r = (uint8_t)(color.r * sinLed_4s[led_sin_4s]);
    color_grb_t.b = (uint8_t)(color.b * sinLed_4s[led_sin_4s]);
    sendLED(color_grb_t, 0);
    led_sin_4s++;

    for (i = 1; i < LED_SUM; i++)
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);

    if (led_sin_4s >= 100)
        led_sin_4s = 0;

    led_sin_2s = 0;
}

void setDiyShow(COLOR color)
{
    int i, ii;
    COLOR color_grb_t;
    color_grb_t.g = (uint8_t)(color.g * sinLed[led_sin_2s]);
    color_grb_t.r = (uint8_t)(color.r * sinLed[led_sin_2s]);
    color_grb_t.b = (uint8_t)(color.b * sinLed[led_sin_2s]);
    sendLED(color_grb_t, 0);
    led_sin_2s++;

    for (i = 1; i < LED_SUM; i++)
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);

    for (ii = 0; ii < 4; ii++)
    {
        if (diyArr[ii][1] != 0)
        {
            for (i = (diyArr[ii][0]); i < (diyArr[ii][0] + diyArr[ii][1]); i++)
            {
                memcpy(&u8TxData[ONE_LED_TYPES * i], &DIY_UINT[diyColor], ONE_LED_TYPES);
            }
        }
    }

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);

    if (led_sin_2s >= 50)
        led_sin_2s = 0;

    led_sin_4s = 0;
}

void setBreath_2s(COLOR color)
{
    int i;
    COLOR color_grb_t;
    color_grb_t.g = (uint8_t)(color.g * sinLed[led_sin_2s]);
    color_grb_t.r = (uint8_t)(color.r * sinLed[led_sin_2s]);
    color_grb_t.b = (uint8_t)(color.b * sinLed[led_sin_2s]);
    sendLED(color_grb_t, 0);
    led_sin_2s++;

    for (i = 1; i < LED_SUM; i++)
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);

    if (led_sin_2s >= 50)
        led_sin_2s = 0;

    led_sin_4s = 0;
}

void setDoubleFlashMode(void)
{
    setNormalRedMode();
    vTaskDelay(100);

    setOffMode();
    vTaskDelay(100);

    setNormalRedMode();
    vTaskDelay(100);

    setOffMode();
    vTaskDelay(650);
}
void setDoubleFlashRed_1Hz_Mode(void)
{
    setNormalRedMode();
    vTaskDelay(500);

    setOffMode();
    vTaskDelay(460);
}
void setDoubleFlashRed_2Hz_Mode(void)
{
    setNormalRedMode();
    vTaskDelay(250);

    setOffMode();
    vTaskDelay(250);

    setNormalRedMode();
    vTaskDelay(250);

    setOffMode();
    vTaskDelay(210);
}

void setDoubleFlash_1Hz_Mode(COLOR color)
{
    setNormalMode(color);
    vTaskDelay(500);

    setOffMode();
    vTaskDelay(460);

    led_sin_2s = 0;
    led_sin_4s = 0;
}

void setDoubleFlash_2Hz_Mode(COLOR color)
{
    setNormalMode(color);
    vTaskDelay(250);

    setOffMode();
    vTaskDelay(250);

    setNormalMode(color);
    vTaskDelay(250);

    setOffMode();
    vTaskDelay(210);

    led_sin_2s = 0;
    led_sin_4s = 0;
}

void setSegmentedMode(COLOR color, COLOR color2, uint8_t sum_header)
{
    int i;
    sendLED(color, 0);

    for (i = 1; i < sum_header; i++)
    {
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[0], ONE_LED_TYPES);
    }

    sendLED(color2, sum_header);

    for (i = sum_header + 1; i < LED_SUM; i++)
    {
        memcpy(&u8TxData[ONE_LED_TYPES * i], &u8TxData[ONE_LED_TYPES * sum_header], ONE_LED_TYPES);
    }

    spi2_send_bytes((uint8_t *)&u8TxData[0u], LED_ARR);
}

#if 1
void ledUpdate(void)
{
    switch (ledMode)
    {
    case OFF_MODE:
    {
        setOffMode();
        break;
    }

    case NORMAL_MODE:
    {
        setNormalMode(color_grb);
        break;
    }

    case FLASH_MODE:
    {
        //  setDoubleFlashMode();
        setDoubleFlashMode();
        break;
    }

    case BREATH_2S:
    {
        setBreath_2s(color_grb);
        break;
    }

    case BREATH_4S:
    {
        setBreath_4s(color_grb);
        break;
    }

    case DIY_SHOW:
    {
        setDiyShow(color_grb);
        break;
    }

    case FLASH_1HZ_MODE:
    {
        setDoubleFlash_1Hz_Mode(color_grb);
        break;
    }

    case FLASH_2HZ_MODE:
    {
        setDoubleFlash_2Hz_Mode(color_grb);
        break;
    }

    default:
        break;
    }
}

#endif

COLOR Colourful_Wheel(u8 WheelPos)
{

    COLOR color;
    WheelPos = 255 - WheelPos;

    if (WheelPos < 85)
    {
        color.r = (uint8_t)((255 - WheelPos * 3) * (rainbow_percent / 100.0));
        color.g = 0;
        color.b = (uint8_t)(WheelPos * 3 * (rainbow_percent / 100.0));
        return color;
    }

    if (WheelPos < 170)
    {
        WheelPos -= 85;
        color.r = 0;
        color.g = (uint8_t)(WheelPos * 3 * (rainbow_percent / 100.0));
        color.b = (uint8_t)((255 - WheelPos * 3) * (rainbow_percent / 100.0));
        return color;
    }

    WheelPos -= 170;
    color.r = (uint8_t)(WheelPos * 3 * (rainbow_percent / 100.0));
    color.g = (uint8_t)((255 - WheelPos * 3) * (rainbow_percent / 100.0));
    color.b = 0;

    return color;
}

void rgb_SetColor(u16 LedId, COLOR Color)
{

    u16 i;

    if (LedId > (LED_SUM))
    {
        //        printf( "Error:Out of Range!\r\n" );
        return; // to avoid overflow
    }

    for (i = 0; i <= 7; i++)
    {
        pixelBuffer[LedId][i] = ((Color.g & (1 << (7 - i))) ? (WS2812_BIT_1) : WS2812_BIT_0);
    }

    for (i = 8; i <= 15; i++)
    {
        pixelBuffer[LedId][i] = ((Color.r & (1 << (15 - i))) ? (WS2812_BIT_1) : WS2812_BIT_0);
    }

    for (i = 16; i <= 23; i++)
    {
        pixelBuffer[LedId][i] = ((Color.b & (1 << (23 - i))) ? (WS2812_BIT_1) : WS2812_BIT_0);
    }
}

void rainbowCycle(void)
{
    static uint16_t j = 0;
    uint8_t i;

    if (j < 1023)
    {
        for (i = 0; i < LED_SUM; i++)
        {
            rgb_SetColor(i, Colourful_Wheel(((i * 256 / LED_SUM) + j) & 255));
        }

        spi2_send_bytes((uint8_t *)&pixelBuffer[0u], LED_ARR);
        j++;
    }

    else
    {
        j = 0;
    }
}

void ws2812_task_function(void *pvParameters)
{
    //    ledMode = NORMAL_MODE;

    while (1)
    {
        if (ledMode == RAINBOW)
        {
            rainbowCycle();
            vTaskDelay(10);
        }

        else
        {
            ledUpdate();
            vTaskDelay(40);
        }
    }
}
