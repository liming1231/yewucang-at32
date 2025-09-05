#include "ax_uart.h"
#include <string.h>
#include "ax_iic.h"
#include "ax_can.h"
#include "ax_flash.h"

const char *usart1_tx_buf = "1234567890abcdefghijklmnopqrstuvwxwz";

struct uart_data uart1_data;

uint8_t ctrl_buff[32] = {0};
volatile uint8_t recvCmdFlag = 0;

uint16_t cmdId = 0;               // usart1_rx_task_function()
can_tx_message_type ctrlData2Can; // usart1_rx_task_function()

void device_ctrl_pins_init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(RUN_LED_CLK, TRUE);
    crm_periph_clock_enable(USBHUB_RST_CLK, TRUE);
    crm_periph_clock_enable(SWITCH_PWR_CLK, TRUE);
    crm_periph_clock_enable(ANDRIOD_PWR_CLK, TRUE);
    crm_periph_clock_enable(CAR_CMPUTER_ACC_CLK, TRUE);

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_init_struct.gpio_pins = RUN_LED_PIN;
    gpio_init(RUN_LED_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = USBHUB_RST_PIN;
    gpio_init(USBHUB_RST_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = SWITCH_PWR_PIN;
    gpio_init(SWITCH_PWR_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = ANDRIOD_PWR_PIN;
    gpio_init(ANDRIOD_PWR_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = CAR_CMPUTER_ACC_PIN;
    gpio_init(CAR_CMPUTER_ACC_PORT, &gpio_init_struct);

    open_device_pwr();
}

void usbhub_ctrl(uint8_t sts)
{
    if (sts == TURN_ON)
    {
        gpio_bits_set(USBHUB_RST_PORT, USBHUB_RST_PIN);
    }
    else
    {
        gpio_bits_reset(USBHUB_RST_PORT, USBHUB_RST_PIN);
    }
}

void switch_pwr_ctrl(uint8_t sts)
{
    if (sts == TURN_ON)
    {
        gpio_bits_set(SWITCH_PWR_PORT, SWITCH_PWR_PIN);
    }
    else
    {
        gpio_bits_reset(SWITCH_PWR_PORT, SWITCH_PWR_PIN);
    }
}

void andriod_pwr_ctrl(uint8_t sts)
{
    if (sts == TURN_ON)
    {
        gpio_bits_set(ANDRIOD_PWR_PORT, ANDRIOD_PWR_PIN);
    }
    else
    {
        gpio_bits_reset(ANDRIOD_PWR_PORT, ANDRIOD_PWR_PIN);
    }
}

void car_acc_ctrl(uint8_t sts)
{
    if (sts == TURN_ON)
    {
        gpio_bits_reset(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
    }
    else
    {
        gpio_bits_set(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
    }
}

void open_device_pwr(void)
{
    usbhub_ctrl(TURN_ON);
    switch_pwr_ctrl(TURN_ON);
    andriod_pwr_ctrl(TURN_ON);
    car_acc_ctrl(TURN_ON);
}

void toggle_led_stat(void)
{
    RUN_LED_PORT->odt ^= RUN_LED_PIN;
}

void init_uart1_data(void)
{
    memcpy(uart1_data.usart1_tx_buffer, usart1_tx_buf, strlen(usart1_tx_buf));
    memset(uart1_data.usart1_rx_buffer, 0, sizeof(uart1_data.usart1_rx_buffer));
    uart1_data.usart1_tx_counter = 0;
    uart1_data.usart1_rx_counter = 0;
    uart1_data.usart1_tx_buffer_size = strlen(usart1_tx_buf);
    uart1_data.usart1_rx_buffer_size = 0;
}

void usart_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the usart2 and gpio clock */
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(UART2_GPIO_CLOCK, TRUE);

    /* enable the usart3 and gpio clock */
    crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(UART3_GPIO_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart2 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = UART2_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(UART2_TX_PORT, &gpio_init_struct);

    /* configure the usart3 tx pin */
    gpio_init_struct.gpio_pins = UART3_TX_PIN;
    gpio_init(UART3_TX_PORT, &gpio_init_struct);

    /* configure the usart2 rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = UART2_RX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(UART2_RX_PORT, &gpio_init_struct);

    /* configure the usart3 rx pin */
    gpio_init_struct.gpio_pins = UART3_RX_PIN;
    gpio_init(UART3_RX_PORT, &gpio_init_struct);

    /* configure usart2 param */
    usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART2, USART_PARITY_NONE);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);
    usart_enable(USART2, TRUE);

    /* configure usart3 param */
    usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART3, USART_PARITY_NONE);
    usart_transmitter_enable(USART3, TRUE);
    usart_receiver_enable(USART3, TRUE);
    usart_enable(USART3, TRUE);
}

void usart_int_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the usart1 and gpio clock */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(UART1_GPIO_CLOCK, TRUE);

    /* enable the usart2 and gpio clock */
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(UART2_GPIO_CLOCK, TRUE);

    /* enable the usart3 and gpio clock */
    crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(UART3_GPIO_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart2 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_init_struct.gpio_pins = UART1_TX_PIN;
    gpio_init(UART1_TX_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = UART2_TX_PIN;
    gpio_init(UART2_TX_PORT, &gpio_init_struct);

    /* configure the usart3 tx pin */
    gpio_init_struct.gpio_pins = UART3_TX_PIN;
    gpio_init(UART3_TX_PORT, &gpio_init_struct);

    /* configure the usart2 rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;

    gpio_init_struct.gpio_pins = UART1_RX_PIN;
    gpio_init(UART1_RX_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = UART2_RX_PIN;
    gpio_init(UART2_RX_PORT, &gpio_init_struct);

    /* configure the usart3 rx pin */
    gpio_init_struct.gpio_pins = UART3_RX_PIN;
    gpio_init(UART3_RX_PORT, &gpio_init_struct);

    /* config usart nvic interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(USART1_IRQn, 0, 0);
    nvic_irq_enable(USART2_IRQn, 0, 0);
    nvic_irq_enable(USART3_IRQn, 0, 0);

    /* configure usart2 param */
    usart_init(USART1, UART_BAUDRATE_115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);

    /* configure usart2 param */
    usart_init(USART2, UART_BAUDRATE_115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);

    /* configure usart3 param */
    usart_init(USART3, UART_BAUDRATE_115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART3, TRUE);
    usart_receiver_enable(USART3, TRUE);

    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);

    /* enable usart2 and usart3 interrupt */
    usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    usart_enable(USART2, TRUE);

    usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
    usart_enable(USART3, TRUE);
    //  usart_interrupt_enable(USART1, USART_TDBE_INT, TRUE);
    //  usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);
    //  usart_interrupt_enable(USART3, USART_TDBE_INT, TRUE);
}

void send_reboot_fb(uint8_t rebootDevFlag)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));

#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 15, "reboot %d succ\r\n", rebootDevFlag);
    uart1_data.usart1_tx_buffer_size = 15;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_REBOOT_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_REBOOT_CMD_ID >> 0) & 0xFF); // 0x01;
    uart1_data.usart1_tx_buffer[5] = (rebootDevFlag == CTRL_OWNER_FLAG) ? 0x00 : rebootDevFlag;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.need_reboot_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif

    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);

    if (rebootDevFlag == CTRL_OWNER_FLAG)
    {
        vTaskDelay(1000);
        nvic_system_reset();
    }
}

void send_distance(void)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 108, "F1 Online(%d): %03d %03d %03d;\tF2 Online(%d): %03d %03d %03d;\tF3 Online(%d): %03d %03d %03d;\tF4 Online(%d): %03d %03d %03d\r\n",
             canAliveCounter.onLine[0],
             distance[0][0], distance[0][1], distance[0][2],
             canAliveCounter.onLine[1],
             distance[1][0], distance[1][1], distance[1][2],
             canAliveCounter.onLine[2],
             distance[2][0], distance[2][1], distance[2][2],
             canAliveCounter.onLine[3],
             distance[3][0], distance[3][1], distance[3][2]);
    uart1_data.usart1_tx_buffer_size = 108;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x22;
    uart1_data.usart1_tx_buffer[3] = 0x05;
    uart1_data.usart1_tx_buffer[4] = 0x00;
    uart1_data.usart1_tx_buffer[5] = 0x01;
    uart1_data.usart1_tx_buffer[6] = canAliveCounter.onLine[0];
    uart1_data.usart1_tx_buffer[7] = distance[0][0];
    uart1_data.usart1_tx_buffer[8] = distance[0][1];
    uart1_data.usart1_tx_buffer[9] = distance[0][2];
    uart1_data.usart1_tx_buffer[10] = distance[0][3];
    uart1_data.usart1_tx_buffer[11] = distance[0][4];
    uart1_data.usart1_tx_buffer[12] = distance[0][5];
    uart1_data.usart1_tx_buffer[13] = 0x02;
    uart1_data.usart1_tx_buffer[14] = canAliveCounter.onLine[1];
    uart1_data.usart1_tx_buffer[15] = distance[1][0];
    uart1_data.usart1_tx_buffer[16] = distance[1][1];
    uart1_data.usart1_tx_buffer[17] = distance[1][2];
    uart1_data.usart1_tx_buffer[18] = distance[1][3];
    uart1_data.usart1_tx_buffer[19] = distance[1][4];
    uart1_data.usart1_tx_buffer[20] = distance[1][5];
    uart1_data.usart1_tx_buffer[21] = 0x03;
    uart1_data.usart1_tx_buffer[22] = canAliveCounter.onLine[2];
    uart1_data.usart1_tx_buffer[23] = distance[2][0];
    uart1_data.usart1_tx_buffer[24] = distance[2][1];
    uart1_data.usart1_tx_buffer[25] = distance[2][2];
    uart1_data.usart1_tx_buffer[26] = distance[2][3];
    uart1_data.usart1_tx_buffer[27] = distance[2][4];
    uart1_data.usart1_tx_buffer[28] = distance[2][5];
    uart1_data.usart1_tx_buffer[29] = 0x04;
    uart1_data.usart1_tx_buffer[30] = canAliveCounter.onLine[3];
    uart1_data.usart1_tx_buffer[31] = distance[3][0];
    uart1_data.usart1_tx_buffer[32] = distance[3][1];
    uart1_data.usart1_tx_buffer[33] = distance[3][2];
    uart1_data.usart1_tx_buffer[34] = distance[3][3];
    uart1_data.usart1_tx_buffer[35] = distance[3][4];
    uart1_data.usart1_tx_buffer[36] = distance[3][5];
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[37] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[38] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 39;
#endif

    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(50);
}
void send_version(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 27, "F%d version:%02X%02X%02X%02X\r\n",
             index + 1, versionSub[index][0], versionSub[index][1], versionSub[index][2], versionSub[index][3]);
    uart1_data.usart1_tx_buffer_size = 27;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x0A;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[5] = index + 1;
    uart1_data.usart1_tx_buffer[6] = versionSub[index][0];
    uart1_data.usart1_tx_buffer[7] = versionSub[index][1];
    uart1_data.usart1_tx_buffer[8] = versionSub[index][2];
    uart1_data.usart1_tx_buffer[9] = versionSub[index][3];
    uart1_data.usart1_tx_buffer[10] = 0x00;
    uart1_data.usart1_tx_buffer[11] = 0x00;
    uart1_data.usart1_tx_buffer[12] = VALID;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[13] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[14] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 15;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_own_version()
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 22, "own version:%08X\r\n", VERSION);
    uart1_data.usart1_tx_buffer_size = 22;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x0A;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[5] = 0x00;
    uart1_data.usart1_tx_buffer[6] = (uint8_t)((VERSION >> 24) & 0xFF);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((VERSION >> 16) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((VERSION >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[9] = (uint8_t)((VERSION >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer[10] = 0x00;
    uart1_data.usart1_tx_buffer[11] = 0x00;
    uart1_data.usart1_tx_buffer[12] = VALID;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[13] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[14] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 15;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 19;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_SET_LEDS_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_SET_LEDS_CMD_ID >> 0) & 0xFF); // 0x03;
    uart1_data.usart1_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.ctrl_leds_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_tray_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));

    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_SET_TRAY_LEDS_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_SET_TRAY_LEDS_CMD_ID >> 0) & 0xFF); // 0x03;
    uart1_data.usart1_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.ctrl_tray_leds;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;

    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_reset_sensor_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 22, "reset F%d sensor succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 22;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 0) & 0xFF); // 0x04;
    uart1_data.usart1_tx_buffer[5] = index;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.reset_sensor_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_usbhub_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 19;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_RESET_USBHUB_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_RESET_USBHUB_CMD_ID >> 0) & 0xFF); // 0x05;
    uart1_data.usart1_tx_buffer[5] = 0x00;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.reset_usbhub_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_switch_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 19;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart1_tx_buffer[5] = 0x00;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.reset_switch_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_andriod_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 19;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart1_tx_buffer[5] = 0x00;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.reset_andriod_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_acc_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));
#ifdef DEBUG
    snprintf(uart1_data.usart1_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart1_data.usart1_tx_buffer_size = 19;
#else
    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x04;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart1_tx_buffer[5] = 0x00;
    uart1_data.usart1_tx_buffer[6] = uart1SendTypeFlag.reset_acc_valid;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 9;
#endif
    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void send_uid(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart1_tx_buffer, 0, sizeof(uart1_data.usart1_tx_buffer));

    uart1_data.usart1_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart1_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart1_tx_buffer[2] = 0x10;
    uart1_data.usart1_tx_buffer[3] = (uint8_t)((FB_GET_UID_CMD_ID >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[4] = (uint8_t)((FB_GET_UID_CMD_ID >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer[5] = index;
    uart1_data.usart1_tx_buffer[6] = uidBuf[index][11];
    uart1_data.usart1_tx_buffer[7] = uidBuf[index][10];
    uart1_data.usart1_tx_buffer[8] = uidBuf[index][9];
    uart1_data.usart1_tx_buffer[9] = uidBuf[index][8];
    uart1_data.usart1_tx_buffer[10] = uidBuf[index][7];
    uart1_data.usart1_tx_buffer[11] = uidBuf[index][6];
    uart1_data.usart1_tx_buffer[12] = uidBuf[index][5];
    uart1_data.usart1_tx_buffer[13] = uidBuf[index][4];
    uart1_data.usart1_tx_buffer[14] = uidBuf[index][3];
    uart1_data.usart1_tx_buffer[15] = uidBuf[index][2];
    uart1_data.usart1_tx_buffer[16] = uidBuf[index][1];
    uart1_data.usart1_tx_buffer[17] = uidBuf[index][0];
    uart1_data.usart1_tx_buffer[18] = VALID;
    getCrc = crc16_modbus(&uart1_data.usart1_tx_buffer[3], uart1_data.usart1_tx_buffer[2]);
    uart1_data.usart1_tx_buffer[19] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart1_tx_buffer[20] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart1_tx_buffer_size = 21;

    while (uart1_data.usart1_tx_counter < uart1_data.usart1_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart1_tx_buffer[uart1_data.usart1_tx_counter++]);
    }

    uart1_data.usart1_tx_counter = 0;
    vTaskDelay(5);
}

void check_version(void)
{
    if (uart1SendTypeFlag.version_own == 1)
    {
        send_own_version();
        uart1SendTypeFlag.version_own = 0;
    }

    if (uart1SendTypeFlag.version_f1 == 1)
    {
        send_version(VERSION_INDEX_F1);
        uart1SendTypeFlag.version_f1 = 0;
    }

    if (uart1SendTypeFlag.version_f2 == 1)
    {
        send_version(VERSION_INDEX_F2);
        uart1SendTypeFlag.version_f2 = 0;
    }

    if (uart1SendTypeFlag.version_f3 == 1)
    {
        send_version(VERSION_INDEX_F3);
        uart1SendTypeFlag.version_f3 = 0;
    }

    if (uart1SendTypeFlag.version_f4 == 1)
    {
        send_version(VERSION_INDEX_F4);
        uart1SendTypeFlag.version_f4 = 0;
    }
}

void check_uid(void)
{
    if (uart1SendTypeFlag.uid_own == 1)
    {
        send_uid(0);
        uart1SendTypeFlag.uid_own = 0;
    }
    if (uart1SendTypeFlag.uid_f1 == 0x03)
    {
        send_uid(1);
        uart1SendTypeFlag.uid_f1 = 0;
    }
    if (uart1SendTypeFlag.uid_f2 == 0x03)
    {
        send_uid(2);
        uart1SendTypeFlag.uid_f2 = 0;
    }
    if (uart1SendTypeFlag.uid_f3 == 0x03)
    {
        send_uid(3);
        uart1SendTypeFlag.uid_f3 = 0;
    }
    if (uart1SendTypeFlag.uid_f4 == 0x03)
    {
        send_uid(4);
        uart1SendTypeFlag.uid_f4 = 0;
    }
}

void check_ctrl_sw_cmd(void)
{
    if (uart1SendTypeFlag.reset_usbhub == 1)
    {
        send_ctrl_usbhub_fb(1);
        uart1SendTypeFlag.reset_usbhub = 0;
    }
    if (uart1SendTypeFlag.reset_switch == 1)
    {
        send_ctrl_switch_fb(1);
        uart1SendTypeFlag.reset_switch = 0;
    }
    if (uart1SendTypeFlag.reset_andriod == 1)
    {
        send_ctrl_andriod_fb(1);
        uart1SendTypeFlag.reset_andriod = 0;
    }
    if (uart1SendTypeFlag.reset_acc == 1)
    {
        send_ctrl_acc_fb(1);
        uart1SendTypeFlag.reset_acc = 0;
    }
}

void usart1_tx_task_function(void *pvParameters)
{
    while (1)
    {
        send_distance();

        if (uart1SendTypeFlag.need_reboot != 0)
        {
            send_reboot_fb(uart1SendTypeFlag.need_reboot);
            uart1SendTypeFlag.need_reboot = 0;
        }

        if (uart1SendTypeFlag.ctrl_leds != 0)
        {
            send_ctrl_leds_fb(uart1SendTypeFlag.ctrl_leds);
            uart1SendTypeFlag.ctrl_leds = 0;
        }

        if (uart1SendTypeFlag.ctrl_tray_leds != 0)
        {
            send_ctrl_tray_leds_fb(uart1SendTypeFlag.ctrl_tray_leds);
            uart1SendTypeFlag.ctrl_tray_leds = 0;
        }

        if (uart1SendTypeFlag.reset_sensor != 0)
        {
            send_reset_sensor_fb(uart1SendTypeFlag.reset_sensor);
            uart1SendTypeFlag.reset_sensor = 0;
        }

        check_version();

        check_uid();

        check_ctrl_sw_cmd();

        taskAliveBits |= TASK_UART_TX_BIT_3;
    }
}

void init_can_msg(void)
{
    ctrlData2Can.extended_id = 0;
    ctrlData2Can.id_type = CAN_ID_STANDARD;
    ctrlData2Can.frame_type = CAN_TFT_DATA;
    ctrlData2Can.dlc = 8;

    canSetLeds[0].extended_id = 0;
    canSetLeds[0].id_type = CAN_ID_STANDARD;
    canSetLeds[0].frame_type = CAN_TFT_DATA;
    canSetLeds[0].dlc = 8;
    canSetLeds[1].extended_id = 0;
    canSetLeds[1].id_type = CAN_ID_STANDARD;
    canSetLeds[1].frame_type = CAN_TFT_DATA;
    canSetLeds[1].dlc = 8;
    canSetLeds[2].extended_id = 0;
    canSetLeds[2].id_type = CAN_ID_STANDARD;
    canSetLeds[2].frame_type = CAN_TFT_DATA;
    canSetLeds[2].dlc = 8;
    canSetLeds[3].extended_id = 0;
    canSetLeds[3].id_type = CAN_ID_STANDARD;
    canSetLeds[3].frame_type = CAN_TFT_DATA;
    canSetLeds[3].dlc = 8;

    canSetTrayLeds.extended_id = 0;
    canSetTrayLeds.id_type = CAN_ID_STANDARD;
    canSetTrayLeds.frame_type = CAN_TFT_DATA;
    canSetTrayLeds.dlc = 8;

    canResetSensor[0].extended_id = 0;
    canResetSensor[0].id_type = CAN_ID_STANDARD;
    canResetSensor[0].frame_type = CAN_TFT_DATA;
    canResetSensor[0].dlc = 8;
    canResetSensor[1].extended_id = 0;
    canResetSensor[1].id_type = CAN_ID_STANDARD;
    canResetSensor[1].frame_type = CAN_TFT_DATA;
    canResetSensor[1].dlc = 8;
    canResetSensor[2].extended_id = 0;
    canResetSensor[2].id_type = CAN_ID_STANDARD;
    canResetSensor[2].frame_type = CAN_TFT_DATA;
    canResetSensor[2].dlc = 8;
    canResetSensor[3].extended_id = 0;
    canResetSensor[3].id_type = CAN_ID_STANDARD;
    canResetSensor[3].frame_type = CAN_TFT_DATA;
    canResetSensor[3].dlc = 8;

    canResetBoard[0].extended_id = 0;
    canResetBoard[0].id_type = CAN_ID_STANDARD;
    canResetBoard[0].frame_type = CAN_TFT_DATA;
    canResetBoard[0].dlc = 8;
    canResetBoard[1].extended_id = 0;
    canResetBoard[1].id_type = CAN_ID_STANDARD;
    canResetBoard[1].frame_type = CAN_TFT_DATA;
    canResetBoard[1].dlc = 8;
    canResetBoard[2].extended_id = 0;
    canResetBoard[2].id_type = CAN_ID_STANDARD;
    canResetBoard[2].frame_type = CAN_TFT_DATA;
    canResetBoard[2].dlc = 8;
    canResetBoard[3].extended_id = 0;
    canResetBoard[3].id_type = CAN_ID_STANDARD;
    canResetBoard[3].frame_type = CAN_TFT_DATA;
    canResetBoard[3].dlc = 8;
}

void usart1_rx_task_function(void *pvParameters)
{
    uint16_t getCrc;

    init_can_msg();

    while (1)
    {
        if (recvCmdFlag == 1)
        {
            // vTaskSuspendAll();
#ifdef DEBUG
            getCrc = 1;
            if (getCrc == 1)
#else
            getCrc = crc16_modbus(&ctrl_buff[3], ctrl_buff[2]);
            if (getCrc == (((ctrl_buff[ctrl_buff[2] + 3] << 8) | ctrl_buff[ctrl_buff[2] + 4]) & 0xFFFF))
#endif
            {
                cmdId = ((ctrl_buff[3] << 8) | ctrl_buff[4]) & 0xFFFF;

                switch (cmdId)
                {
                case REBOOT_CMD_ID:
                {
                    if (ctrl_buff[5] > TRAY_SUM)
                    {
                        uart1SendTypeFlag.need_reboot_valid = 0;
                        uart1SendTypeFlag.need_reboot = ctrl_buff[5] == 0x00 ? CTRL_OWNER_FLAG : ctrl_buff[5];
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        uart1SendTypeFlag.need_reboot_valid = 1;
                        uart1SendTypeFlag.need_reboot = CTRL_OWNER_FLAG;
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        canResetBoard[0].standard_id = SET_F1_REBOOT_ID;
                        canResetBoard[0].data[0] = 0x01;
                        canResetBoard[0].data[1] = 0x00;
                        canResetBoard[0].data[2] = 0x00;
                        canResetBoard[0].data[3] = 0x00;
                        canResetBoard[0].data[4] = 0x00;
                        canResetBoard[0].data[5] = 0x00;
                        canResetBoard[0].data[6] = 0x00;
                        sendCanStr.resetBoard[0] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        canResetBoard[1].standard_id = SET_F2_REBOOT_ID;
                        canResetBoard[1].data[0] = 0x01;
                        canResetBoard[1].data[1] = 0x00;
                        canResetBoard[1].data[2] = 0x00;
                        canResetBoard[1].data[3] = 0x00;
                        canResetBoard[1].data[4] = 0x00;
                        canResetBoard[1].data[5] = 0x00;
                        canResetBoard[1].data[6] = 0x00;
                        sendCanStr.resetBoard[1] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        canResetBoard[2].standard_id = SET_F3_REBOOT_ID;
                        canResetBoard[2].data[0] = 0x01;
                        canResetBoard[2].data[1] = 0x00;
                        canResetBoard[2].data[2] = 0x00;
                        canResetBoard[2].data[3] = 0x00;
                        canResetBoard[2].data[4] = 0x00;
                        canResetBoard[2].data[5] = 0x00;
                        canResetBoard[2].data[6] = 0x00;
                        sendCanStr.resetBoard[2] = 1;
                    }
                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        canResetBoard[3].standard_id = SET_F4_REBOOT_ID;
                        canResetBoard[3].data[0] = 0x01;
                        canResetBoard[3].data[1] = 0x00;
                        canResetBoard[3].data[2] = 0x00;
                        canResetBoard[3].data[3] = 0x00;
                        canResetBoard[3].data[4] = 0x00;
                        canResetBoard[3].data[5] = 0x00;
                        canResetBoard[3].data[6] = 0x00;
                        sendCanStr.resetBoard[3] = 1;
                    }

                    break;
                }

                case GET_VERSION_CMD_ID:
                {
                    if (ctrl_buff[5] > TRAY_SUM)
                    {
                        // 无效
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        uart1SendTypeFlag.version_own = 1;
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        ctrlData2Can.standard_id = GET_F1_VERSION_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        ctrlData2Can.standard_id = GET_F2_VERSION_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        ctrlData2Can.standard_id = GET_F3_VERSION_ID;
                    }
                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        ctrlData2Can.standard_id = GET_F4_VERSION_ID;
                    }

                    ctrlData2Can.data[0] = 0x01;
                    ctrlData2Can.data[1] = 0x00;
                    ctrlData2Can.data[2] = 0x00;
                    ctrlData2Can.data[3] = 0x00;
                    ctrlData2Can.data[4] = 0x00;
                    ctrlData2Can.data[5] = 0x00;
                    ctrlData2Can.data[6] = 0x00;
                    can_transmit_ctrl_data(&ctrlData2Can);
                    break;
                }

                case SET_LEDS_CMD_ID:
                {
                    if ((ctrl_buff[5] > TRAY_SUM) || (ctrl_buff[6] > LED_MODE_MAX) || (ctrl_buff[7] > 0x80) || (ctrl_buff[8] > 0x80) || (ctrl_buff[9] > 0x80))
                    {
                        uart1SendTypeFlag.ctrl_leds_valid = 0;
                        uart1SendTypeFlag.ctrl_leds = ctrl_buff[5] == 0x00 ? 0x0f : ctrl_buff[5];
                        break;
                    }
                    else if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] > 0x64))
                    {
                        uart1SendTypeFlag.ctrl_leds_valid = 0;
                        uart1SendTypeFlag.ctrl_leds = ctrl_buff[5];
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] <= 0x64))
                        {
                            ledMode = ctrl_buff[6];
                            rainbow_percent = ctrl_buff[7];
                        }
                        ledMode = ctrl_buff[6];
                        color_grb.r = ctrl_buff[7];
                        color_grb.g = ctrl_buff[8];
                        color_grb.b = ctrl_buff[9];
                        uart1SendTypeFlag.ctrl_leds_valid = 1;
                        uart1SendTypeFlag.ctrl_leds = 0x0f;
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        canSetLeds[0].standard_id = SET_F1_WS2812B_ID;

                        canSetLeds[0].data[0] = ctrl_buff[6];
                        canSetLeds[0].data[1] = ctrl_buff[7];
                        canSetLeds[0].data[2] = ctrl_buff[8];
                        canSetLeds[0].data[3] = ctrl_buff[9];
                        canSetLeds[0].data[4] = 0x00;
                        canSetLeds[0].data[5] = 0x00;
                        canSetLeds[0].data[6] = 0x00;
                        sendCanStr.setWs2812b[0] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        canSetLeds[1].standard_id = SET_F2_WS2812B_ID;
                        canSetLeds[1].data[0] = ctrl_buff[6];
                        canSetLeds[1].data[1] = ctrl_buff[7];
                        canSetLeds[1].data[2] = ctrl_buff[8];
                        canSetLeds[1].data[3] = ctrl_buff[9];
                        canSetLeds[1].data[4] = 0x00;
                        canSetLeds[1].data[5] = 0x00;
                        canSetLeds[1].data[6] = 0x00;
                        sendCanStr.setWs2812b[1] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        canSetLeds[2].standard_id = SET_F3_WS2812B_ID;
                        canSetLeds[2].data[0] = ctrl_buff[6];
                        canSetLeds[2].data[1] = ctrl_buff[7];
                        canSetLeds[2].data[2] = ctrl_buff[8];
                        canSetLeds[2].data[3] = ctrl_buff[9];
                        canSetLeds[2].data[4] = 0x00;
                        canSetLeds[2].data[5] = 0x00;
                        canSetLeds[2].data[6] = 0x00;
                        sendCanStr.setWs2812b[2] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        canSetLeds[3].standard_id = SET_F4_WS2812B_ID;
                        canSetLeds[3].data[0] = ctrl_buff[6];
                        canSetLeds[3].data[1] = ctrl_buff[7];
                        canSetLeds[3].data[2] = ctrl_buff[8];
                        canSetLeds[3].data[3] = ctrl_buff[9];
                        canSetLeds[3].data[4] = 0x00;
                        canSetLeds[3].data[5] = 0x00;
                        canSetLeds[3].data[6] = 0x00;
                        sendCanStr.setWs2812b[3] = 1;
                    }

                    break;
                }
#if 1
                case SET_TRAY_LEDS_CMD_ID:
                {
                    if ((ctrl_buff[6] > LED_MODE_MAX) || (ctrl_buff[7] > 0x80) || (ctrl_buff[8] > 0x80) || (ctrl_buff[9] > 0x80))
                    {
                        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
                        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5] == 0x00 ? 0x0f : ctrl_buff[5];
                        break;
                    }
                    else if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] > 0x64))
                    {
                        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
                        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5];
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        //                        ledMode = ctrl_buff[6];
                        //                        color_grb.r = ctrl_buff[7];
                        //                        color_grb.g = ctrl_buff[8];
                        //                        color_grb.b = ctrl_buff[9];
                        //                        uart1SendTypeFlag.ctrl_tray_leds_valid = 1;
                        //                        uart1SendTypeFlag.ctrl_tray_leds = 0x0f;
                        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
                        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5];
                        break;
                    }
                    else
                    {
                        canSetTrayLeds.standard_id = SET_TRAY_WS2812B_ID;

                        canSetTrayLeds.data[0] = ctrl_buff[6];
                        canSetTrayLeds.data[1] = ctrl_buff[7];
                        canSetTrayLeds.data[2] = ctrl_buff[8];
                        canSetTrayLeds.data[3] = ctrl_buff[9];
                        canSetTrayLeds.data[4] = ctrl_buff[5];
                        canSetTrayLeds.data[5] = 0x00;
                        canSetTrayLeds.data[6] = 0x00;
                        sendCanStr.setTrayWs2812b = 1;
                    }

                    break;
                }
#endif
                case RESET_SENSOR_CMD_ID:
                {
                    if ((ctrl_buff[5] > TRAY_SUM) || (ctrl_buff[5] == TRAY_MASTER))
                    {
                        uart1SendTypeFlag.reset_sensor_valid = 0;
                        uart1SendTypeFlag.reset_sensor = ctrl_buff[5];
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        canResetSensor[0].standard_id = RESET_F1_SENSOR_ID;
                        canResetSensor[0].data[0] = 0x01;
                        canResetSensor[0].data[1] = 0x00;
                        canResetSensor[0].data[2] = 0x00;
                        canResetSensor[0].data[3] = 0x00;
                        canResetSensor[0].data[4] = 0x00;
                        canResetSensor[0].data[5] = 0x00;
                        canResetSensor[0].data[6] = 0x00;
                        sendCanStr.resetSensor[0] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        canResetSensor[1].standard_id = RESET_F2_SENSOR_ID;
                        canResetSensor[1].data[0] = 0x01;
                        canResetSensor[1].data[1] = 0x00;
                        canResetSensor[1].data[2] = 0x00;
                        canResetSensor[1].data[3] = 0x00;
                        canResetSensor[1].data[4] = 0x00;
                        canResetSensor[1].data[5] = 0x00;
                        canResetSensor[1].data[6] = 0x00;
                        sendCanStr.resetSensor[1] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        canResetSensor[2].standard_id = RESET_F3_SENSOR_ID;
                        canResetSensor[2].data[0] = 0x01;
                        canResetSensor[2].data[1] = 0x00;
                        canResetSensor[2].data[2] = 0x00;
                        canResetSensor[2].data[3] = 0x00;
                        canResetSensor[2].data[4] = 0x00;
                        canResetSensor[2].data[5] = 0x00;
                        canResetSensor[2].data[6] = 0x00;
                        sendCanStr.resetSensor[2] = 1;
                    }

                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        canResetSensor[3].standard_id = RESET_F4_SENSOR_ID;
                        canResetSensor[3].data[0] = 0x01;
                        canResetSensor[3].data[1] = 0x00;
                        canResetSensor[3].data[2] = 0x00;
                        canResetSensor[3].data[3] = 0x00;
                        canResetSensor[3].data[4] = 0x00;
                        canResetSensor[3].data[5] = 0x00;
                        canResetSensor[3].data[6] = 0x00;
                        sendCanStr.resetSensor[3] = 1;
                    }

                    break;
                }

                case RESET_USBHUB_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        uart1SendTypeFlag.reset_usbhub_valid = 0;
                        uart1SendTypeFlag.reset_usbhub = 0x01;
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            usbhub_ctrl(0);
                            uart1SendTypeFlag.reset_usbhub_valid = 1;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            usbhub_ctrl(1);
                            uart1SendTypeFlag.reset_usbhub_valid = 1;
                        }
                        else
                        {
                            uart1SendTypeFlag.reset_usbhub_valid = 0;
                        }
                        uart1SendTypeFlag.reset_usbhub = 0x01;
                    }

                    break;
                }
                case RESET_SWITCH_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        uart1SendTypeFlag.reset_switch_valid = 0x00;
                        uart1SendTypeFlag.reset_switch = 0x01;
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            switch_pwr_ctrl(0);
                            uart1SendTypeFlag.reset_switch_valid = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            switch_pwr_ctrl(1);
                            uart1SendTypeFlag.reset_switch_valid = 0x01;
                        }
                        else
                        {
                            uart1SendTypeFlag.reset_switch_valid = 0x00;
                        }
                        uart1SendTypeFlag.reset_switch = 0x01;
                    }

                    break;
                }
                case RESET_ANDRIOD_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        uart1SendTypeFlag.reset_andriod_valid = 0x00;
                        uart1SendTypeFlag.reset_andriod = 0x01;
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            andriod_pwr_ctrl(0);
                            uart1SendTypeFlag.reset_andriod_valid = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            andriod_pwr_ctrl(1);
                            uart1SendTypeFlag.reset_andriod_valid = 0x01;
                        }
                        else
                        {
                            uart1SendTypeFlag.reset_andriod_valid = 0x00;
                        }
                        uart1SendTypeFlag.reset_andriod = 0x01;
                    }
                    break;
                }
                case RESET_ACC_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        uart1SendTypeFlag.reset_acc_valid = 0x00;
                        uart1SendTypeFlag.reset_acc = 0x01;
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            car_acc_ctrl(0);
                            uart1SendTypeFlag.reset_acc_valid = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            car_acc_ctrl(1);
                            uart1SendTypeFlag.reset_acc_valid = 0x01;
                        }
                        else
                        {
                            uart1SendTypeFlag.reset_acc_valid = 0x00;
                        }
                        uart1SendTypeFlag.reset_acc = 0x01;
                    }
                    break;
                }
                case GET_UID_CMD_ID:
                {
                    if (ctrl_buff[5] > TRAY_SUM)
                    {
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        uart1SendTypeFlag.uid_own = 1;
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        ctrlData2Can.standard_id = GET_F1_UID_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        ctrlData2Can.standard_id = GET_F2_UID_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        ctrlData2Can.standard_id = GET_F3_UID_ID;
                    }
                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        ctrlData2Can.standard_id = GET_F4_UID_ID;
                    }

                    ctrlData2Can.data[0] = 0x01;
                    ctrlData2Can.data[1] = 0x00;
                    ctrlData2Can.data[2] = 0x00;
                    ctrlData2Can.data[3] = 0x00;
                    ctrlData2Can.data[4] = 0x00;
                    ctrlData2Can.data[5] = 0x00;
                    ctrlData2Can.data[6] = 0x00;
                    can_transmit_ctrl_data(&ctrlData2Can);
                    break;
                }

                default:
                {
                    break;
                }
                }
            }
            // xTaskResumeAll();
            recvCmdFlag = 0;
        }
        taskAliveBits |= TASK_UART_RX_BIT_2;
        vTaskDelay(5);
    }
}

void USART1_IRQHandler(void)
{
    static uint8_t revFlag = UART_DATA_INIT;

    if (usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    {
        usart_flag_clear(USART1, USART_RDBF_FLAG);
        if (recvCmdFlag == 0)
        {
            if (revFlag == UART_DATA_INIT)
            {
                if (usart_data_receive(USART1) == UART_MSG_HEADER_1)
                {
                    revFlag = UART_GET_HEADER_1;
                }
            }

            else if (revFlag == UART_GET_HEADER_1)
            {
                if (usart_data_receive(USART1) == UART_MSG_HEADER_2)
                {
                    revFlag = UART_GET_HEADER_2;
                }
            }

            else if (revFlag == UART_GET_HEADER_2)
            {
                uart1_data.usart1_rx_buffer[0] = UART_MSG_HEADER_1;
                uart1_data.usart1_rx_buffer[1] = UART_MSG_HEADER_2;
                uart1_data.usart1_rx_buffer[2] = usart_data_receive(USART1);

                if (uart1_data.usart1_rx_buffer[2] <= UART_MSG_DATA_LEN_MAX)
                {
                    uart1_data.usart1_rx_counter = 3;
                    revFlag = UART_GET_MSG_DATA_LEN;
                }
            }

            else if (revFlag == UART_GET_MSG_DATA_LEN)
            {
                uart1_data.usart1_rx_buffer[uart1_data.usart1_rx_counter++] = usart_data_receive(USART1);

                if (uart1_data.usart1_rx_counter == uart1_data.usart1_rx_buffer[2] + 5)
                {
                    memcpy(ctrl_buff, uart1_data.usart1_rx_buffer, uart1_data.usart1_rx_counter);
                    recvCmdFlag = 1;
                    memset(uart1_data.usart1_rx_buffer, 0, sizeof(uart1_data.usart1_rx_buffer));
                    uart1_data.usart1_rx_counter = 0;
                    uart1_data.usart1_rx_buffer_size = 0;
                    revFlag = UART_DATA_INIT;
                }
            }

            /* read one byte from the receive data register */
            //      uart1_data.usart1_rx_buffer[uart1_data.usart1_rx_counter++] = usart_data_receive( USART1 );

            //      if( uart1_data.usart1_rx_counter == 6 )
            //      {
            /* disable the usart2 receive interrupt */
            // s  usart_interrupt_enable(USART1, USART_RDBF_INT, FALSE);

            //      }
        }
    }

    if (usart_flag_get(USART1, USART_TDBE_FLAG) != RESET)
    {
        // USART_TDC_FLAG;
    }
}
