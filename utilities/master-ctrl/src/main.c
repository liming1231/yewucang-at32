#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "ax_uart.h"
#include "ax_can.h"
#include "ax_spi.h"
#include "ax_ws2812.h"
#include "ax_flash.h"

/** @addtogroup UTILITIES_examples
 * @{
 */
#define LED2_STK_SIZE (16)
#define UART1_RX_STK_SIZE (512)
#define UART1_TX_STK_SIZE (1024)
// #define I2C1_STK_SIZE     (128)
#define CAN_TX_STK_SIZE (256)
#define CAN_RX_STK_SIZE (256)
#define WTDG_STK_SIZE (128)
#define WS2812B_STK_SIZE (512)

#define LED2_TASK_PRIO (2)
#define UART1_RX_TASK_PRIO (2)
#define UART1_TX_TASK_PRIO (2)
#define CAN_TX_TASK_PRIO (2)
#define CAN_RX_TASK_PRIO (2)
#define WTDG_TASK_PRIO (6)
#define WS2812B_TASK_PRIO (5)

/** @addtogroup FreeRTOS_demo
 * @{
 */
TaskHandle_t led2_handler;
TaskHandle_t usart1_rx_handler;
TaskHandle_t usart1_tx_handler;
TaskHandle_t i2c1_handler;
TaskHandle_t can_tx_handler;
TaskHandle_t can_rx_handler;
TaskHandle_t ws2812_handler;
TaskHandle_t wtdg_handler;
// EventGroupHandle_t xCreatedEventGroup;//事件组句柄

/* led2 task */
void led2_task_function(void *pvParameters);
void iwdg_Config(void);
void wtdg_task_function(void *pvParameters);

/**
 * @brief  main function.
 * @param  none
 * @retval none
 */
int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    system_clock_config();
    // delay_init();

    flash_read_u8(0x1FFFF7E8, uidBuf[0], 12);

    device_ctrl_pins_init();
    open_device_pwr();

    init_uart1_data();
    usart_int_configuration();
    /* init usart1 */
    // uart_print_init( UART_BAUDRATE_115200 );

    can_alive_struct_init();
    can_gpio_config();
    can_configuration();
    canbus_open();

    ws2812Init();

    //    xCreatedEventGroup = xEventGroupCreate();	//�����¼���

    /* enter critical */
    taskENTER_CRITICAL();

    /* create led2 task */
    if (xTaskCreate((TaskFunction_t)led2_task_function,
                    (const char *)"LED2_task",
                    (uint16_t)LED2_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)LED2_TASK_PRIO,
                    (TaskHandle_t *)&led2_handler) != pdPASS)
    {
        // printf( "LED2 task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "LED2 task was created successfully.\r\n" );
    }

    /* create led3 task */
    if (xTaskCreate((TaskFunction_t)usart1_rx_task_function,
                    (const char *)"usart1_rx_task",
                    (uint16_t)UART1_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART1_RX_TASK_PRIO,
                    (TaskHandle_t *)&usart1_rx_handler) != pdPASS)
    {
        // printf( "LED3 task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "LED3 task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)usart1_tx_task_function,
                    (const char *)"usart1_tx_task",
                    (uint16_t)UART1_TX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART1_TX_TASK_PRIO,
                    (TaskHandle_t *)&usart1_tx_handler) != pdPASS)
    {
        // printf( "usart2_function task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "usart2_function task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)can_tx_task_function,
                    (const char *)"can_tx_task",
                    (uint16_t)CAN_TX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)CAN_TX_TASK_PRIO,
                    (TaskHandle_t *)&can_tx_handler) != pdPASS)
    {
        // printf( "can tx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "can tx task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)can_rx_task_function,
                    (const char *)"can_rx_task",
                    (uint16_t)CAN_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)CAN_RX_TASK_PRIO,
                    (TaskHandle_t *)&can_rx_handler) != pdPASS)
    {
        // printf( "can rx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "can rx task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)ws2812_task_function,
                    (const char *)"ws2812_task",
                    (uint16_t)WS2812B_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)WS2812B_TASK_PRIO,
                    (TaskHandle_t *)&ws2812_handler) != pdPASS)
    {
        // printf( "ws2812 task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "ws2812 task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)wtdg_task_function,
                    (const char *)"wtdg_task",
                    (uint16_t)WTDG_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)WTDG_TASK_PRIO,
                    (TaskHandle_t *)&wtdg_handler) != pdPASS)
    {
        // printf( "watchdog task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        // printf( "watchdog task was created successfully.\r\n" );
    }

    /* exit critical */
    taskEXIT_CRITICAL();

    /* start scheduler */
    vTaskStartScheduler();
}

/* led task function */
void led2_task_function(void *pvParameters)
{
    while (1)
    {
        toggle_led_stat();
        //      xEventGroupSetBits(xCreatedEventGroup, TASK_DEBUG_LED_BIT_1);
        taskAliveBits |= TASK_DEBUG_LED_BIT_1;

        vTaskDelay(500);
    }
}

#if 1

void wtdg_task_function(void *pvParameters)
{

    //	 const TickType_t xTicksToWait = 0;//2000 / portTICK_PERIOD_MS; /* ����ӳ�2000ms */

    iwdg_Config(); // 5s timeout
    while (1)
    {
        //        wdt_counter_reload();
        /* 等待任务的事件组信号量 */
        //		taskAliveBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件组信号量 */
        //									 TASK_BIT_ALL,       /* 等待TASK_BIT_ALL信号量 */
        //									 pdTRUE,             /* 退出前TASK_BIT_ALL信号量必须被清除 */
        //									 pdTRUE,             /* 等待为pdTRUE则表示等待TASK_BIT_ALL信号量 */
        //									 xTicksToWait);      /* 等待时间 */
        if ((taskAliveBits & TASK_BIT_ALL) == TASK_BIT_ALL) // 判断任务是否执行
        {
            // xEventGroupClearBits(xCreatedEventGroup, taskAliveBits);
            taskAliveBits = 0;
            wdt_counter_reload();
        }
        vTaskDelay(5);
    }
}
#endif

void iwdg_Config()
{
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        /* reset from wdt */
        crm_flag_clear(CRM_WDT_RESET_FLAG);
    }
    /* disable register write protection */
    wdt_register_write_enable(TRUE);

    /* set the wdt divider value */
    wdt_divider_set(WDT_CLK_DIV_128);

    /* set reload value

    timeout = reload_value * (divider / lick_freq )    (s)

    lick_freq    = 40000 Hz
    divider      = 128
    reload_value = 10000

    timeout = 10000 * (4 / 40000 ) = 0.3s = 300ms
    */
    wdt_reload_value_set(1500 - 1);
    /* reload wdt counter */
    wdt_counter_reload();
    /* enable wdt */
    wdt_enable();
}

/**
 * @}
 */

/**
 * @}
 */
