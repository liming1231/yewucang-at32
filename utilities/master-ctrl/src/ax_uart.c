#include "ax_uart.h"
#include <string.h>
#include "ax_iic.h"
#include "ax_can.h"

const char *usart2_tx_buf = "1234567890abcdefghijklmnopqrstuvwxwz";

struct uart_data uart2_data;

uint8_t ctrl_buff[32] = {0};
volatile uint8_t recvCmdFlag = 0;

uint16_t cmdId = 0;               // usart2_rx_task_function()
can_tx_message_type ctrlData2Can; // usart2_rx_task_function()

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
        gpio_bits_reset(ANDRIOD_PWR_PORT, ANDRIOD_PWR_PIN);
    }
    else
    {
        gpio_bits_set(ANDRIOD_PWR_PORT, ANDRIOD_PWR_PIN);
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

void init_uart2_data(void)
{
    memcpy(uart2_data.usart2_tx_buffer, usart2_tx_buf, strlen(usart2_tx_buf));
    memset(uart2_data.usart2_rx_buffer, 0, sizeof(uart2_data.usart2_rx_buffer));
    uart2_data.usart2_tx_counter = 0;
    uart2_data.usart2_rx_counter = 0;
    uart2_data.usart2_tx_buffer_size = strlen(usart2_tx_buf);
    uart2_data.usart2_rx_buffer_size = 0;
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

    /* config usart nvic interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(USART2_IRQn, 0, 0);
    nvic_irq_enable(USART3_IRQn, 0, 0);

    /* configure usart2 param */
    usart_init(USART2, UART_BAUDRATE_115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);

    /* configure usart3 param */
    usart_init(USART3, UART_BAUDRATE_115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART3, TRUE);
    usart_receiver_enable(USART3, TRUE);

    /* enable usart2 and usart3 interrupt */
    usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    usart_enable(USART2, TRUE);

    usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
    usart_enable(USART3, TRUE);

    //  usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);
    //  usart_interrupt_enable(USART3, USART_TDBE_INT, TRUE);
}

void send_reboot_fb(uint8_t rebootDevFlag)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));

#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 15, "reboot %d succ\r\n", rebootDevFlag);
    uart2_data.usart2_tx_buffer_size = 15;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_REBOOT_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_REBOOT_CMD_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart2_tx_buffer[5] = (rebootDevFlag == CTRL_OWNER_FLAG) ? 0x00 : rebootDevFlag;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif

    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
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
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 108, "F1 Online(%d): %03d %03d %03d;\tF2 Online(%d): %03d %03d %03d;\tF3 Online(%d): %03d %03d %03d;\tF4 Online(%d): %03d %03d %03d\r\n",
             canAliveCounter.onLine[0],
             distance[0][0], distance[0][1], distance[0][2],
             canAliveCounter.onLine[1],
             distance[1][0], distance[1][1], distance[1][2],
             canAliveCounter.onLine[2],
             distance[2][0], distance[2][1], distance[2][2],
             canAliveCounter.onLine[3],
             distance[3][0], distance[3][1], distance[3][2]);
    uart2_data.usart2_tx_buffer_size = 108;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x22;
    uart2_data.usart2_tx_buffer[3] = 0x05;
    uart2_data.usart2_tx_buffer[4] = 0x00;
    uart2_data.usart2_tx_buffer[5] = 0x01;
    uart2_data.usart2_tx_buffer[6] = canAliveCounter.onLine[0];
    uart2_data.usart2_tx_buffer[7] = distance[0][0];
    uart2_data.usart2_tx_buffer[8] = distance[0][1];
    uart2_data.usart2_tx_buffer[9] = distance[0][2];
    uart2_data.usart2_tx_buffer[10] = distance[0][3];
    uart2_data.usart2_tx_buffer[11] = distance[0][4];
    uart2_data.usart2_tx_buffer[12] = distance[0][5];
    uart2_data.usart2_tx_buffer[13] = 0x02;
    uart2_data.usart2_tx_buffer[14] = canAliveCounter.onLine[1];
    uart2_data.usart2_tx_buffer[15] = distance[1][0];
    uart2_data.usart2_tx_buffer[16] = distance[1][1];
    uart2_data.usart2_tx_buffer[17] = distance[1][2];
    uart2_data.usart2_tx_buffer[18] = distance[1][3];
    uart2_data.usart2_tx_buffer[19] = distance[1][4];
    uart2_data.usart2_tx_buffer[20] = distance[1][5];
    uart2_data.usart2_tx_buffer[21] = 0x03;
    uart2_data.usart2_tx_buffer[22] = canAliveCounter.onLine[2];
    uart2_data.usart2_tx_buffer[23] = distance[2][0];
    uart2_data.usart2_tx_buffer[24] = distance[2][1];
    uart2_data.usart2_tx_buffer[25] = distance[2][2];
    uart2_data.usart2_tx_buffer[26] = distance[2][3];
    uart2_data.usart2_tx_buffer[27] = distance[2][4];
    uart2_data.usart2_tx_buffer[28] = distance[2][5];
    uart2_data.usart2_tx_buffer[29] = 0x04;
    uart2_data.usart2_tx_buffer[30] = canAliveCounter.onLine[3];
    uart2_data.usart2_tx_buffer[31] = distance[3][0];
    uart2_data.usart2_tx_buffer[32] = distance[3][1];
    uart2_data.usart2_tx_buffer[33] = distance[3][2];
    uart2_data.usart2_tx_buffer[34] = distance[3][3];
    uart2_data.usart2_tx_buffer[35] = distance[3][4];
    uart2_data.usart2_tx_buffer[36] = distance[3][5];
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[37] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[38] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 39;
#endif

    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(50);
}
void send_version(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 27, "F%d version:%02X%02X%02X%02X\r\n",
             index + 1, versionSub[index][0], versionSub[index][1], versionSub[index][2], versionSub[index][3]);
    uart2_data.usart2_tx_buffer_size = 27;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x0A;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[5] = index + 1;
    uart2_data.usart2_tx_buffer[6] = versionSub[index][0];
    uart2_data.usart2_tx_buffer[7] = versionSub[index][1];
    uart2_data.usart2_tx_buffer[8] = versionSub[index][2];
    uart2_data.usart2_tx_buffer[9] = versionSub[index][3];
    uart2_data.usart2_tx_buffer[10] = 0x00;
    uart2_data.usart2_tx_buffer[11] = 0x00;
    uart2_data.usart2_tx_buffer[12] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[13] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[14] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 15;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_own_version()
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 22, "own version:%08X\r\n", VERSION);
    uart2_data.usart2_tx_buffer_size = 22;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x0A;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[5] = 0x00;
    uart2_data.usart2_tx_buffer[6] = (uint8_t)((VERSION >> 24) & 0xFF);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((VERSION >> 16) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((VERSION >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[9] = (uint8_t)((VERSION >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer[10] = 0x00;
    uart2_data.usart2_tx_buffer[11] = 0x00;
    uart2_data.usart2_tx_buffer[12] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[13] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[14] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 15;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 19;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_SET_LEDS_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_SET_LEDS_CMD_ID >> 0) & 0xFF); // 0x03;
    uart2_data.usart2_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_reset_sensor_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 22, "reset F%d sensor succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 22;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 0) & 0xFF); // 0x04;
    uart2_data.usart2_tx_buffer[5] = index;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_usbhub_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 19;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_RESET_USBHUB_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_RESET_USBHUB_CMD_ID >> 0) & 0xFF); // 0x05;
    uart2_data.usart2_tx_buffer[5] = 0x00;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_switch_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 19;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 0) & 0xFF); // 0x06;
    uart2_data.usart2_tx_buffer[5] = 0x00;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_andriod_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 19;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 0) & 0xFF); // 0x06;
    uart2_data.usart2_tx_buffer[5] = 0x00;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_acc_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart2_data.usart2_tx_buffer, 0, sizeof(uart2_data.usart2_tx_buffer));
#ifdef DEBUG
    snprintf(uart2_data.usart2_tx_buffer, 19, "ctrl F%d leds succ\r\n", index);
    uart2_data.usart2_tx_buffer_size = 19;
#else
    uart2_data.usart2_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart2_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart2_tx_buffer[2] = 0x04;
    uart2_data.usart2_tx_buffer[3] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart2_tx_buffer[4] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 0) & 0xFF); // 0x06;
    uart2_data.usart2_tx_buffer[5] = 0x00;
    uart2_data.usart2_tx_buffer[6] = VALID;
    getCrc = crc16_modbus(&uart2_data.usart2_tx_buffer[3], uart2_data.usart2_tx_buffer[2]);
    uart2_data.usart2_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart2_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart2_tx_buffer_size = 9;
#endif
    while (uart2_data.usart2_tx_counter < uart2_data.usart2_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart2_tx_buffer[uart2_data.usart2_tx_counter++]);
    }

    uart2_data.usart2_tx_counter = 0;
    vTaskDelay(5);
}

void check_version(void)
{
    if (uart2SendTypeFlag.version_own == 1)
    {
        send_own_version();
        uart2SendTypeFlag.version_own = 0;
    }

    if (uart2SendTypeFlag.version_f1 == 1)
    {
        send_version(VERSION_INDEX_F1);
        uart2SendTypeFlag.version_f1 = 0;
    }

    if (uart2SendTypeFlag.version_f2 == 1)
    {
        send_version(VERSION_INDEX_F2);
        uart2SendTypeFlag.version_f2 = 0;
    }

    if (uart2SendTypeFlag.version_f3 == 1)
    {
        send_version(VERSION_INDEX_F3);
        uart2SendTypeFlag.version_f3 = 0;
    }

    if (uart2SendTypeFlag.version_f4 == 1)
    {
        send_version(VERSION_INDEX_F4);
        uart2SendTypeFlag.version_f4 = 0;
    }
}

void check_ctrl_sw_cmd(void)
{
    if (uart2SendTypeFlag.reset_usbhub == 1)
    {
        send_ctrl_usbhub_fb(1);
        uart2SendTypeFlag.reset_usbhub = 0;
    }
    if (uart2SendTypeFlag.reset_switch == 1)
    {
        send_ctrl_switch_fb(1);
        uart2SendTypeFlag.reset_switch = 0;
    }
    if (uart2SendTypeFlag.reset_andriod == 1)
    {
        send_ctrl_andriod_fb(1);
        uart2SendTypeFlag.reset_andriod = 0;
    }
    if (uart2SendTypeFlag.reset_acc == 1)
    {
        send_ctrl_acc_fb(1);
        uart2SendTypeFlag.reset_acc = 0;
    }
}

void usart2_tx_task_function(void *pvParameters)
{
    while (1)
    {
        send_distance();

        if (uart2SendTypeFlag.need_reboot != 0)
        {
            send_reboot_fb(uart2SendTypeFlag.need_reboot);
            uart2SendTypeFlag.need_reboot = 0;
        }

        if (uart2SendTypeFlag.ctrl_leds != 0)
        {
            send_ctrl_leds_fb(uart2SendTypeFlag.ctrl_leds);
            uart2SendTypeFlag.ctrl_leds = 0;
        }

        if (uart2SendTypeFlag.reset_sensor != 0)
        {
            send_reset_sensor_fb(uart2SendTypeFlag.reset_sensor);
            uart2SendTypeFlag.reset_sensor = 0;
        }

        check_version();

        check_ctrl_sw_cmd();

        taskAliveBits |= TASK_UART_TX_BIT_3;
    }
}

void usart2_rx_task_function(void *pvParameters)
{
    uint16_t getCrc;
    ctrlData2Can.extended_id = 0;
    ctrlData2Can.id_type = CAN_ID_STANDARD;
    ctrlData2Can.frame_type = CAN_TFT_DATA;
    ctrlData2Can.dlc = 8;

    while (1)
    {
#if 1
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
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        uart2SendTypeFlag.need_reboot = CTRL_OWNER_FLAG;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        ctrlData2Can.standard_id = SET_F1_REBOOT_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        ctrlData2Can.standard_id = SET_F2_REBOOT_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        ctrlData2Can.standard_id = SET_F3_REBOOT_ID;
                    }
                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        ctrlData2Can.standard_id = SET_F4_REBOOT_ID;
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

                case GET_VERSION_CMD_ID:
                {
                    if (ctrl_buff[5] > TRAY_SUM)
                    {
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        uart2SendTypeFlag.version_own = 1;
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
                    if (ctrl_buff[5] > TRAY_SUM)
                    {
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_MASTER)
                    {
                        ledMode = ctrl_buff[6];
                        color_grb.r = ctrl_buff[7];
                        color_grb.g = ctrl_buff[8];
                        color_grb.b = ctrl_buff[9];
                        uart2SendTypeFlag.ctrl_leds = 0x0f;
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        ctrlData2Can.standard_id = SET_F1_WS2812B_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        ctrlData2Can.standard_id = SET_F2_WS2812B_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        ctrlData2Can.standard_id = SET_F3_WS2812B_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        ctrlData2Can.standard_id = SET_F4_WS2812B_ID;
                    }

                    ctrlData2Can.data[0] = ctrl_buff[6];
                    ctrlData2Can.data[1] = ctrl_buff[7];
                    ctrlData2Can.data[2] = ctrl_buff[8];
                    ctrlData2Can.data[3] = ctrl_buff[9];
                    ctrlData2Can.data[4] = 0x00;
                    ctrlData2Can.data[5] = 0x00;
                    ctrlData2Can.data[6] = 0x00;
                    can_transmit_ctrl_data(&ctrlData2Can);
                    break;
                }

                case RESET_SENSOR_CMD_ID:
                {
                    if ((ctrl_buff[5] > TRAY_SUM) || (ctrl_buff[5] == TRAY_MASTER))
                    {
                        break;
                    }

                    else if (ctrl_buff[5] == TRAY_F1)
                    {
                        ctrlData2Can.standard_id = RESET_F1_SENSOR_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F2)
                    {
                        ctrlData2Can.standard_id = RESET_F2_SENSOR_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F3)
                    {
                        ctrlData2Can.standard_id = RESET_F3_SENSOR_ID;
                    }

                    else if (ctrl_buff[5] == TRAY_F4)
                    {
                        ctrlData2Can.standard_id = RESET_F4_SENSOR_ID;
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

                case RESET_USBHUB_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            usbhub_ctrl(0);
                            uart2SendTypeFlag.reset_usbhub = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            usbhub_ctrl(1);
                            uart2SendTypeFlag.reset_usbhub = 0x01;
                        }
                    }
                    break;
                }
                case RESET_SWITCH_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            switch_pwr_ctrl(0);
                            uart2SendTypeFlag.reset_switch = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            switch_pwr_ctrl(1);
                            uart2SendTypeFlag.reset_switch = 0x01;
                        }
                    }
                    break;
                }
                case RESET_ANDRIOD_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            andriod_pwr_ctrl(0);
                            uart2SendTypeFlag.reset_andriod = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            andriod_pwr_ctrl(1);
                            uart2SendTypeFlag.reset_andriod = 0x01;
                        }
                    }
                    break;
                }
                case RESET_ACC_CMD_ID:
                {
                    if (ctrl_buff[5] != 0x00)
                    {
                        break;
                    }
                    else
                    {
                        if (ctrl_buff[6] == 0x00)
                        {
                            car_acc_ctrl(0);
                            uart2SendTypeFlag.reset_acc = 0x01;
                        }
                        else if (ctrl_buff[6] == 0x01)
                        {
                            car_acc_ctrl(1);
                            uart2SendTypeFlag.reset_acc = 0x01;
                        }
                    }
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

#else
        // for debug
        uint8_t onlineIndex = 0;
        if (canAliveCounter.onLine[0] == 0x01)
        {
            ctrlData2Can.standard_id = SET_F1_WS2812B_ID;
            onlineIndex = 1;
        }
        else if (canAliveCounter.onLine[1] == 0x01)
        {
            ctrlData2Can.standard_id = SET_F2_WS2812B_ID;
            onlineIndex = 2;
        }
        else if (canAliveCounter.onLine[2] == 0x01)
        {
            ctrlData2Can.standard_id = SET_F3_WS2812B_ID;
            onlineIndex = 3;
        }
        else if (canAliveCounter.onLine[3] == 0x01)
        {
            ctrlData2Can.standard_id = SET_F4_WS2812B_ID;
            onlineIndex = 4;
        }

        ctrlData2Can.data[0] = 1;
        ctrlData2Can.data[1] = 0x00;
        ctrlData2Can.data[2] = 0x00;
        ctrlData2Can.data[3] = 0x00;
        if ((distance[onlineIndex - 1][0] < 25) && ((distance[onlineIndex - 1][0] > 0)))
        {
            ctrlData2Can.data[1] |= 0xFF;
        }
        if ((distance[onlineIndex - 1][1] < 25) && ((distance[onlineIndex - 1][1] > 0)))
        {
            ctrlData2Can.data[2] |= 0xFF;
        }
        if ((distance[onlineIndex - 1][2] < 25) && ((distance[onlineIndex - 1][2] > 0)))
        {
            ctrlData2Can.data[3] |= 0xFF;
        }
        ctrlData2Can.data[4] = 0x00;
        ctrlData2Can.data[5] = 0x00;
        ctrlData2Can.data[6] = 0x00;
        can_transmit_ctrl_data(&ctrlData2Can);
        taskAliveBits |= TASK_UART_RX_BIT_2;
        vTaskDelay(50);
#endif
    }
}

void USART2_IRQHandler(void)
{
    static uint8_t revFlag = UART_DATA_INIT;

    if (usart_flag_get(USART2, USART_RDBF_FLAG) != RESET)
    {
        if (recvCmdFlag == 0)
        {
            if (revFlag == UART_DATA_INIT)
            {
                if (usart_data_receive(USART2) == UART_MSG_HEADER_1)
                {
                    revFlag = UART_GET_HEADER_1;
                }
            }

            else if (revFlag == UART_GET_HEADER_1)
            {
                if (usart_data_receive(USART2) == UART_MSG_HEADER_2)
                {
                    revFlag = UART_GET_HEADER_2;
                }
            }

            else if (revFlag == UART_GET_HEADER_2)
            {
                uart2_data.usart2_rx_buffer[0] = UART_MSG_HEADER_1;
                uart2_data.usart2_rx_buffer[1] = UART_MSG_HEADER_2;
                uart2_data.usart2_rx_buffer[2] = usart_data_receive(USART2);

                if (uart2_data.usart2_rx_buffer[2] <= UART_MSG_DATA_LEN_MAX)
                {
                    uart2_data.usart2_rx_counter = 3;
                    revFlag = UART_GET_MSG_DATA_LEN;
                }
            }

            else if (revFlag == UART_GET_MSG_DATA_LEN)
            {
                uart2_data.usart2_rx_buffer[uart2_data.usart2_rx_counter++] = usart_data_receive(USART2);

                if (uart2_data.usart2_rx_counter == uart2_data.usart2_rx_buffer[2] + 5)
                {
                    memcpy(ctrl_buff, uart2_data.usart2_rx_buffer, uart2_data.usart2_rx_counter);
                    recvCmdFlag = 1;
                    memset(uart2_data.usart2_rx_buffer, 0, sizeof(uart2_data.usart2_rx_buffer));
                    uart2_data.usart2_rx_counter = 0;
                    uart2_data.usart2_rx_buffer_size = 0;
                    revFlag = UART_DATA_INIT;
                }
            }

            /* read one byte from the receive data register */
            //      uart2_data.usart2_rx_buffer[uart2_data.usart2_rx_counter++] = usart_data_receive( USART2 );

            //      if( uart2_data.usart2_rx_counter == 6 )
            //      {
            /* disable the usart2 receive interrupt */
            // s  usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);

            //      }
        }
    }

    if (usart_flag_get(USART2, USART_TDBE_FLAG) != RESET)
    {
        // USART_TDC_FLAG;
    }
}
