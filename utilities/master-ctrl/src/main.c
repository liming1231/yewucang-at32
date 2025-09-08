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
#define UART2_RX_STK_SIZE (128)
#define UART1_CMD_STK_SIZE (128)

#define LED2_TASK_PRIO (2)
#define UART1_RX_TASK_PRIO (2)
#define UART1_TX_TASK_PRIO (2)
#define CAN_TX_TASK_PRIO (2)
#define CAN_RX_TASK_PRIO (2)
#define WTDG_TASK_PRIO (6)
#define WS2812B_TASK_PRIO (5)
#define UART2_RX_TASK_PRIO (2)
#define UART1_CMD_TASK_PRIO (3)

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
TaskHandle_t usart2_rx_handler;
TaskHandle_t usart1_cmd_parse_handler;
// EventGroupHandle_t xCreatedEventGroup;//�����¼���

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
    updateFw.runningAppAddrFlag = *(uint8_t *)(START_APP_FLAG_ADDR);

    if (updateFw.runningAppAddrFlag == APP2_FLAG)
    {
        updateFw.runningAppBaseAddr = RUNNING_APP2_ADDR;
        updateFw.updateAppBaseAddr = RUNNING_APP1_ADDR;
        updateFw.updateAppAddr = RUNNING_APP1_ADDR;
        updateFw.updateAppAddrFlag = APP1_FLAG;
        nvic_vector_table_set(NVIC_VECTTAB_FLASH, RUNNING_APP2_ADDR - ROM_BASE_ADDR);
        // SCB->VTOR = 0x801C000;
    }

    else
    {
        updateFw.runningAppBaseAddr = RUNNING_APP1_ADDR;
        updateFw.updateAppBaseAddr = RUNNING_APP2_ADDR;
        updateFw.updateAppAddr = RUNNING_APP2_ADDR;
        updateFw.updateAppAddrFlag = APP2_FLAG;
        nvic_vector_table_set(NVIC_VECTTAB_FLASH, RUNNING_APP1_ADDR - ROM_BASE_ADDR);
        // SCB->VTOR = 0x8004000;
    }

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    system_clock_config();
    // delay_init();

    flash_read_u8(0x1FFFF7E8, uidBuf[0], 12);

    device_ctrl_pins_init();
    open_device_pwr();

    init_uart_data();
    usart_int_configuration();
    /* init usart1 */
    //    uart_print_init( UART_BAUDRATE_115200 );

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
        //        printf( "LED2 task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "LED2 task was created successfully.\r\n" );
    }

    /* create uart1 rx task */
    if (xTaskCreate((TaskFunction_t)usart1_rx_task_function,
                    (const char *)"usart1_rx_task",
                    (uint16_t)UART1_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART1_RX_TASK_PRIO,
                    (TaskHandle_t *)&usart1_rx_handler) != pdPASS)
    {
        //        printf( "uart1 rx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "uart1 rx was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)usart1_tx_task_function,
                    (const char *)"usart1_tx_task",
                    (uint16_t)UART1_TX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART1_TX_TASK_PRIO,
                    (TaskHandle_t *)&usart1_tx_handler) != pdPASS)
    {
        //        printf( "usart2_function task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "usart2_function task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)can_tx_task_function,
                    (const char *)"can_tx_task",
                    (uint16_t)CAN_TX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)CAN_TX_TASK_PRIO,
                    (TaskHandle_t *)&can_tx_handler) != pdPASS)
    {
        //        printf( "can tx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "can tx task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)can_rx_task_function,
                    (const char *)"can_rx_task",
                    (uint16_t)CAN_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)CAN_RX_TASK_PRIO,
                    (TaskHandle_t *)&can_rx_handler) != pdPASS)
    {
        //       printf( "can rx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "can rx task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)ws2812_task_function,
                    (const char *)"ws2812_task",
                    (uint16_t)WS2812B_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)WS2812B_TASK_PRIO,
                    (TaskHandle_t *)&ws2812_handler) != pdPASS)
    {
        //        printf( "ws2812 task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "ws2812 task was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)wtdg_task_function,
                    (const char *)"wtdg_task",
                    (uint16_t)WTDG_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)WTDG_TASK_PRIO,
                    (TaskHandle_t *)&wtdg_handler) != pdPASS)
    {
        //        printf( "watchdog task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "watchdog task was created successfully.\r\n" );
    }

    /* create uart2 rx task */
    if (xTaskCreate((TaskFunction_t)usart2_rx_task_function,
                    (const char *)"usart2_rx_task",
                    (uint16_t)UART2_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART2_RX_TASK_PRIO,
                    (TaskHandle_t *)&usart2_rx_handler) != pdPASS)
    {
        //        printf( "uart2 rx task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "uart2 rx was created successfully.\r\n" );
    }

    if (xTaskCreate((TaskFunction_t)usart1_cmd_parse_task_function,
                    (const char *)"usart1 cmd parse task",
                    (uint16_t)UART2_RX_STK_SIZE,
                    (void *)NULL,
                    (UBaseType_t)UART2_RX_TASK_PRIO,
                    (TaskHandle_t *)&usart1_cmd_parse_handler) != pdPASS)
    {
        //        printf( "uart1 cmd parse task could not be created as there was insufficient heap memory remaining.\r\n" );
    }

    else
    {
        //        printf( "uart1 cmd parse was created successfully.\r\n" );
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
        /* �ȴ������������¼���־ */
        //		taskAliveBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
        //									 TASK_BIT_ALL,       /* �ȴ�TASK_BIT_ALL������ */
        //									 pdTRUE,             /* �˳�ǰTASK_BIT_ALL�������������TASK_BIT_ALL�������òű�ʾ���˳���*/
        //									 pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�TASK_BIT_ALL��������*/
        //									 xTicksToWait);      /* �ȴ��ӳ�ʱ�� */
        if ((taskAliveBits & TASK_BIT_ALL) == TASK_BIT_ALL) // �жϸ��������Ƿ�ִ��
        {
            //            xEventGroupClearBits(xCreatedEventGroup, taskAliveBits);
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
