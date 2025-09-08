#include "ax_uart.h"
#include <string.h>
#include "ax_iic.h"
#include "ax_can.h"
#include "ax_flash.h"

const char *usart1_tx_buf = "1234567890abcdefghijklmnopqrstuvwxwz";
volatile uint8_t sendingUart2 = 0;

struct uart_data uart1_data, uart2_data;

static uint8_t diyShowDataSts = 1;

uint8_t fb_ctrl_buf[32] = {0};
uint8_t ws2812b_ctrl_buf[32] = {0};

uint8_t ctrl_buff[32] = {0};
uint8_t ctrl_buff2[256] = {0};
uint8_t update_buff[16][128] = {0};
volatile uint8_t recvCmdFlag = 0;
volatile uint8_t recvCmdFlag2 = 0;

uint16_t cmdId = 0;               // usart1_rx_task_function()
uint16_t cmdId2 = 0;              // usart1_rx_task_function()
can_tx_message_type ctrlData2Can; // usart1_rx_task_function()

void device_ctrl_pins_init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(RUN_LED_CLK, TRUE);
    crm_periph_clock_enable(USBHUB_RST_CLK, TRUE);
    crm_periph_clock_enable(SWITCH_PWR_CLK, TRUE);
    crm_periph_clock_enable(ANDRIOD_PWR_CLK, TRUE);
    crm_periph_clock_enable(CAR_CMPUTER_ACC_CLK, TRUE);

    crm_periph_clock_enable(IHAWK_POWER_1_CLK, TRUE);
    crm_periph_clock_enable(IHAWK_POWER_2_CLK, TRUE);
    crm_periph_clock_enable(USB3_POWER_CLK, TRUE);
    gpio_pin_remap_config(SWJTAG_MUX_010, TRUE); // 只保留 SWD，释放 PB4

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

    gpio_init_struct.gpio_pins = IHAWK_POWER_1_PIN;
    gpio_init(IHAWK_POWER_1_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = IHAWK_POWER_2_PIN;
    gpio_init(IHAWK_POWER_2_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = USB3_POWER_PIN;
    gpio_init(USB3_POWER_PORT, &gpio_init_struct);

    gpio_bits_reset(IHAWK_POWER_1_PORT, IHAWK_POWER_1_PIN);
    gpio_bits_reset(IHAWK_POWER_2_PORT, IHAWK_POWER_2_PIN);
    gpio_bits_reset(USB3_POWER_PORT, USB3_POWER_PIN);

    //    open_device_pwr();
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
    else if (sts == TURN_OFF)
    {
        gpio_bits_reset(ANDRIOD_PWR_PORT, ANDRIOD_PWR_PIN);
    }
}

void car_acc_ctrl(uint8_t sts)
{
    if (sts == TURN_ON)
    {
        // gpio_bits_reset(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
        gpio_bits_set(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
    }
    else
    {
        // gpio_bits_set(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
        gpio_bits_reset(CAR_CMPUTER_ACC_PORT, CAR_CMPUTER_ACC_PIN);
    }
}

void parse_ctrl_andriod(uint8_t mode)
{
    if (mode < 0x02)
    {
        andriod_pwr_ctrl(mode);
        uart2SendTypeFlag.reset_andriod_valid = 0x01;
    }
    else
    {
        uart2SendTypeFlag.reset_andriod_valid = 0x00;
    }
    uart2SendTypeFlag.reset_andriod = 0x01;
}
void parse_ctrl_andriod_tm(uint8_t mode, uint8_t delayTm)
{
    if (mode == 0x03)
    {
        andriod_pwr_ctrl(0);
        vTaskDelay(delayTm * 100);
        andriod_pwr_ctrl(1);
        uart2SendTypeFlag.reset_andriod_valid = 0x01;
    }
    else
    {
        uart2SendTypeFlag.reset_andriod_valid = 0x00;
    }
    uart2SendTypeFlag.reset_andriod = 0x01;
}

void ihawk_lower_ctrl(uint8_t sts)
{
    if (sts == TURN_OFF)
    {
        gpio_bits_reset(IHAWK_POWER_1_PORT, IHAWK_POWER_1_PIN);
        ihawk_power_sts.ihawk_sts_1 = 0;
    }
    else
    {
        gpio_bits_set(IHAWK_POWER_1_PORT, IHAWK_POWER_1_PIN);
        ihawk_power_sts.ihawk_sts_1 = 1;
    }
}

void ihawk_upper_ctrl(uint8_t sts)
{
    if (sts == TURN_OFF)
    {
        gpio_bits_reset(IHAWK_POWER_2_PORT, IHAWK_POWER_2_PIN);
        ihawk_power_sts.ihawk_sts_2 = 0;
    }
    else
    {
        gpio_bits_set(IHAWK_POWER_2_PORT, IHAWK_POWER_2_PIN);
        ihawk_power_sts.ihawk_sts_2 = 1;
    }
}

void usb3_ctrl(uint8_t sts)
{
    if (sts == TURN_OFF)
    {
        gpio_bits_reset(USB3_POWER_PORT, USB3_POWER_PIN);
        ihawk_power_sts.usb3_sts = 0;
    }
    else
    {
        gpio_bits_set(USB3_POWER_PORT, USB3_POWER_PIN);
        ihawk_power_sts.usb3_sts = 1;
    }
}

void ihawk_ctrl(uint8_t index, uint8_t sts)
{
    if (sts <= 2) // enum CTRL_ACTION Type
    {
        switch (index)
        {
        case LOWER_USB: // 下USB口
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                vTaskDelay(200);
                ihawk_lower_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
            }
            break;
        }
        case UPPER_USB: // 上USB口
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_upper_ctrl(TURN_OFF);
                vTaskDelay(200);
                ihawk_upper_ctrl(TURN_ON);
            }
            else
            {
                ihawk_upper_ctrl(sts);
            }
            break;
        }
        case DUAL_USB: // 同时控制双USB
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                ihawk_upper_ctrl(TURN_OFF);
                vTaskDelay(200);
                ihawk_lower_ctrl(TURN_ON);
                ihawk_upper_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
                ihawk_upper_ctrl(sts);
            }
            break;
        }
        case USB3_NEW:
        {
            if (sts == RESET_IHAWK)
            {
                usb3_ctrl(TURN_OFF);
                vTaskDelay(200);
                usb3_ctrl(TURN_ON);
            }
            else
            {
                usb3_ctrl(sts);
            }
            break;
        }
        case USB_ALL:
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                ihawk_upper_ctrl(TURN_OFF);
                usb3_ctrl(TURN_OFF);
                vTaskDelay(200);
                ihawk_lower_ctrl(TURN_ON);
                ihawk_upper_ctrl(TURN_ON);
                usb3_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
                ihawk_upper_ctrl(sts);
                usb3_ctrl(sts);
            }
            break;
        }
        default:
            break;
        }
    }
}

void ihawk_ctrl_tm(uint8_t index, uint8_t sts, uint8_t delayTm)
{
    if (sts <= 2) // enum CTRL_ACTION Type
    {
        switch (index)
        {
        case LOWER_USB: // 下USB口
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                vTaskDelay(delayTm * 100);
                ihawk_lower_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
            }
            break;
        }
        case UPPER_USB: // 上USB口
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_upper_ctrl(TURN_OFF);
                vTaskDelay(delayTm * 100);
                ihawk_upper_ctrl(TURN_ON);
            }
            else
            {
                ihawk_upper_ctrl(sts);
            }
            break;
        }
        case DUAL_USB: // 同时控制双USB
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                ihawk_upper_ctrl(TURN_OFF);
                vTaskDelay(delayTm * 100);
                ihawk_lower_ctrl(TURN_ON);
                ihawk_upper_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
                ihawk_upper_ctrl(sts);
            }
            break;
        }
        case USB3_NEW:
        {
            if (sts == RESET_IHAWK)
            {
                usb3_ctrl(TURN_OFF);
                vTaskDelay(delayTm * 100);
                usb3_ctrl(TURN_ON);
            }
            else
            {
                usb3_ctrl(sts);
            }
            break;
        }
        case USB_ALL:
        {
            if (sts == RESET_IHAWK)
            {
                ihawk_lower_ctrl(TURN_OFF);
                ihawk_upper_ctrl(TURN_OFF);
                usb3_ctrl(TURN_OFF);
                vTaskDelay(delayTm * 100);
                ihawk_lower_ctrl(TURN_ON);
                ihawk_upper_ctrl(TURN_ON);
                usb3_ctrl(TURN_ON);
            }
            else
            {
                ihawk_lower_ctrl(sts);
                ihawk_upper_ctrl(sts);
                usb3_ctrl(sts);
            }
            break;
        }
        default:
            break;
        }
    }
}

void open_device_pwr(void)
{
    uint64_t ii = 0;
    usbhub_ctrl(TURN_ON);
    switch_pwr_ctrl(TURN_ON);
    andriod_pwr_ctrl(TURN_ON);
    for (ii = 0; ii < 2000000; ii++)
    {
        __NOP();
    }
    car_acc_ctrl(TURN_ON);
    ihawk_ctrl(LOWER_USB, TURN_ON);
    ihawk_ctrl(UPPER_USB, TURN_ON);
    ihawk_ctrl(USB3_NEW, TURN_ON);
}

void toggle_led_stat(void)
{
    RUN_LED_PORT->odt ^= RUN_LED_PIN;
}

void init_uart_data(void)
{
    memcpy(uart1_data.usart_tx_buffer, usart1_tx_buf, strlen(usart1_tx_buf));
    memset(uart1_data.usart_rx_buffer, 0, sizeof(uart1_data.usart_rx_buffer));
    uart1_data.usart_tx_counter = 0;
    uart1_data.usart_rx_counter = 0;
    uart1_data.usart_tx_buffer_size = strlen(usart1_tx_buf);
    uart1_data.usart_rx_buffer_size = 0;

    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));
    memset(uart2_data.usart_rx_buffer, 0, sizeof(uart2_data.usart_rx_buffer));
    uart2_data.usart_tx_counter = 0;
    uart2_data.usart_rx_counter = 0;
    uart2_data.usart_tx_buffer_size = 0;
    uart2_data.usart_rx_buffer_size = 0;
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

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    /* configure the usart1 tx pin */
    gpio_init_struct.gpio_pins = UART1_TX_PIN;
    gpio_init(UART1_TX_PORT, &gpio_init_struct);
    /* configure the usart2 tx pin */
    gpio_init_struct.gpio_pins = UART2_TX_PIN;
    gpio_init(UART2_TX_PORT, &gpio_init_struct);
    /* configure the usart3 tx pin */
    gpio_init_struct.gpio_pins = UART3_TX_PIN;
    gpio_init(UART3_TX_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;

    /* configure the usart1 rx pin */
    gpio_init_struct.gpio_pins = UART1_RX_PIN;
    gpio_init(UART1_RX_PORT, &gpio_init_struct);
    /* configure the usart2 rx pin */
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

    /* configure usart1 param */
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

void send_ctrl_ihawk_fb(uint8_t index, uint8_t sts, uint8_t valid_sts)
{
    uint16_t getCrc;

    while (sendingUart2 == 1)
    {
        vTaskDelay(2);
    }
    sendingUart2 = 1;

    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x06;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_SET_IHAWK_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_SET_IHAWK_CMD_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart_tx_buffer[5] = 0x00;
    uart2_data.usart_tx_buffer[6] = index;
    uart2_data.usart_tx_buffer[7] = sts;
    uart2_data.usart_tx_buffer[8] = valid_sts;
    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[9] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 11;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    sendingUart2 = 0;
    vTaskDelay(5);
}

void send_andriod_fb(uint8_t sts, uint8_t valid_sts)
{
    uint16_t tmCnt = 0;
    uint16_t getCrc;

    while (sendingUart2 == 1)
    {
        vTaskDelay(2);
        tmCnt++;
        if (tmCnt >= 50)
        {
            break;
        }
    }
    sendingUart2 = 1;

    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x06;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart_tx_buffer[5] = 0x00;
    uart2_data.usart_tx_buffer[6] = 0x00;
    uart2_data.usart_tx_buffer[7] = sts;
    uart2_data.usart_tx_buffer[8] = valid_sts;
    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[9] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 11;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    sendingUart2 = 0;
    vTaskDelay(5);
}

void send_ihawk_sts_fb(uint8_t valid_sts)
{
    uint16_t getCrc;

    while (sendingUart2 == 1)
    {
        vTaskDelay(2);
    }
    sendingUart2 = 1;

    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x06;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_IHAWK_STS_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_IHAWK_STS_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart_tx_buffer[5] = 0x00;
    if (valid_sts == 1)
    {
        uart2_data.usart_tx_buffer[6] = ihawk_power_sts.ihawk_sts_1;
        uart2_data.usart_tx_buffer[7] = ihawk_power_sts.ihawk_sts_2;
        uart2_data.usart_tx_buffer[8] = valid_sts;
    }
    else
    {
        uart2_data.usart_tx_buffer[6] = 0x00;
        uart2_data.usart_tx_buffer[7] = 0x00;
        uart2_data.usart_tx_buffer[8] = 0x00;
    }

    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[9] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 11;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    sendingUart2 = 0;
    vTaskDelay(5);
}

void send_usb_sts_fb(uint8_t valid_sts)
{
    uint16_t getCrc;

    while (sendingUart2 == 1)
    {
        vTaskDelay(2);
    }
    sendingUart2 = 1;

    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x07;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_USB_STS_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_USB_STS_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart_tx_buffer[5] = 0x00;
    if (valid_sts == 1)
    {
        uart2_data.usart_tx_buffer[6] = ihawk_power_sts.ihawk_sts_1;
        uart2_data.usart_tx_buffer[7] = ihawk_power_sts.ihawk_sts_2;
        uart2_data.usart_tx_buffer[8] = ihawk_power_sts.usb3_sts;
        uart2_data.usart_tx_buffer[9] = valid_sts;
    }
    else
    {
        uart2_data.usart_tx_buffer[6] = 0x00;
        uart2_data.usart_tx_buffer[7] = 0x00;
        uart2_data.usart_tx_buffer[8] = 0x00;
        uart2_data.usart_tx_buffer[9] = 0x00;
    }

    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[11] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 12;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    sendingUart2 = 0;
    vTaskDelay(5);
}

void send_reboot_fb(uint8_t rebootDevFlag)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

#ifdef DEBUG
    snprintf(uart1_data.usart_tx_buffer, 15, "reboot %d succ\r\n", rebootDevFlag);
    uart1_data.usart_tx_buffer_size = 15;
#else
    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_REBOOT_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_REBOOT_CMD_ID >> 0) & 0xFF); // 0x01;
    uart1_data.usart_tx_buffer[5] = (rebootDevFlag == CTRL_OWNER_FLAG) ? 0x00 : rebootDevFlag;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.need_reboot_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;
#endif

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);

    if (rebootDevFlag == CTRL_OWNER_FLAG)
    {
        vTaskDelay(1000);
        nvic_system_reset();
    }
}

void send_reboot2_fb(uint8_t cmdType)
{
    uint16_t getCrc;
    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));
    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x04;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_REBOOT_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_REBOOT_CMD_ID >> 0) & 0xFF); // 0x01;
    uart2_data.usart_tx_buffer[5] = 0x00;
    uart2_data.usart_tx_buffer[6] = cmdType;
    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 9;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    vTaskDelay(1000);
    nvic_system_reset();
}

void send_distance(void)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x22;
    uart1_data.usart_tx_buffer[3] = 0x05;
    uart1_data.usart_tx_buffer[4] = 0x00;
    uart1_data.usart_tx_buffer[5] = 0x01;
    uart1_data.usart_tx_buffer[6] = canAliveCounter.onLine[0];
    uart1_data.usart_tx_buffer[7] = distance[0][0];
    uart1_data.usart_tx_buffer[8] = distance[0][1];
    uart1_data.usart_tx_buffer[9] = distance[0][2];
    uart1_data.usart_tx_buffer[10] = distance[0][3];
    uart1_data.usart_tx_buffer[11] = distance[0][4];
    uart1_data.usart_tx_buffer[12] = distance[0][5];
    uart1_data.usart_tx_buffer[13] = 0x02;
    uart1_data.usart_tx_buffer[14] = canAliveCounter.onLine[1];
    uart1_data.usart_tx_buffer[15] = distance[1][0];
    uart1_data.usart_tx_buffer[16] = distance[1][1];
    uart1_data.usart_tx_buffer[17] = distance[1][2];
    uart1_data.usart_tx_buffer[18] = distance[1][3];
    uart1_data.usart_tx_buffer[19] = distance[1][4];
    uart1_data.usart_tx_buffer[20] = distance[1][5];
    uart1_data.usart_tx_buffer[21] = 0x03;
    uart1_data.usart_tx_buffer[22] = canAliveCounter.onLine[2];
    uart1_data.usart_tx_buffer[23] = distance[2][0];
    uart1_data.usart_tx_buffer[24] = distance[2][1];
    uart1_data.usart_tx_buffer[25] = distance[2][2];
    uart1_data.usart_tx_buffer[26] = distance[2][3];
    uart1_data.usart_tx_buffer[27] = distance[2][4];
    uart1_data.usart_tx_buffer[28] = distance[2][5];
    uart1_data.usart_tx_buffer[29] = 0x04;
    uart1_data.usart_tx_buffer[30] = canAliveCounter.onLine[3];
    uart1_data.usart_tx_buffer[31] = distance[3][0];
    uart1_data.usart_tx_buffer[32] = distance[3][1];
    uart1_data.usart_tx_buffer[33] = distance[3][2];
    uart1_data.usart_tx_buffer[34] = distance[3][3];
    uart1_data.usart_tx_buffer[35] = distance[3][4];
    uart1_data.usart_tx_buffer[36] = distance[3][5];
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[37] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[38] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 39;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(50);
}

void send_sts_loop(void)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));
    uartStsData_str.header[0] = 0x55;
    uartStsData_str.header[1] = 0xAA;
    uartStsData_str.dataLen = 0x11;
    uartStsData_str.type = 0x10;
    uartStsData_str.touchPadSts = 0x00;
    uartStsData_str.LockSts_L = 0x00;
    uartStsData_str.LockSts_H = 0x00;
    uartStsData_str.LockSts_Food = 0x00;
    uartStsData_str.Motor_L[0] = 0x00;
    uartStsData_str.Motor_L[1] = 0x00;
    uartStsData_str.Motor_L[2] = 0x00;
    uartStsData_str.Motor_L[3] = 0x00;
    uartStsData_str.Motor_H[0] = 0x00;
    uartStsData_str.Motor_H[1] = 0x00;
    uartStsData_str.Motor_H[2] = 0x00;
    uartStsData_str.Motor_H[3] = 0x00;
    uartStsData_str.ws2812Mode.mode = ledMode;
    uartStsData_str.ws2812Mode.r = color_grb.r;
    uartStsData_str.ws2812Mode.g = color_grb.g;
    uartStsData_str.ws2812Mode.b = color_grb.b;
    memcpy(uart1_data.usart_tx_buffer, (void *)&uartStsData_str, sizeof(uartStsData_str));
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[20] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[21] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 22;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(50);
}

void send_version(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x0B;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[5] = index + 1;
    uart1_data.usart_tx_buffer[6] = VALID;
    uart1_data.usart_tx_buffer[7] = versionSub[index][0];
    uart1_data.usart_tx_buffer[8] = 0x2E;
    uart1_data.usart_tx_buffer[9] = versionSub[index][1];
    uart1_data.usart_tx_buffer[10] = 0x2E;
    uart1_data.usart_tx_buffer[11] = versionSub[index][2];
    uart1_data.usart_tx_buffer[12] = 0x2E;
    uart1_data.usart_tx_buffer[13] = versionSub[index][3];

    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[14] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[15] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 16;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_own_version_old()
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x07;
    uart1_data.usart_tx_buffer[3] = 0x01;
    uart1_data.usart_tx_buffer[4] = fb_ctrl_buf[4];
    uart1_data.usart_tx_buffer[5] = fb_ctrl_buf[5];

    uart1_data.usart_tx_buffer[6] = (uint8_t)((VERSION >> 24) & 0xFF);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((VERSION >> 16) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((VERSION >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[9] = (uint8_t)((VERSION >> 0) & 0xFF);
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[11] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 12;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_own_version()
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x0B;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[5] = 0x00;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.version_own_valid;
    uart1_data.usart_tx_buffer[7] = (uint8_t)((VERSION >> 24) & 0xFF);
    uart1_data.usart_tx_buffer[8] = 0x2E;
    uart1_data.usart_tx_buffer[9] = (uint8_t)((VERSION >> 16) & 0xFF);
    uart1_data.usart_tx_buffer[10] = 0x2E;
    uart1_data.usart_tx_buffer[11] = (uint8_t)((VERSION >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[12] = 0x2E;
    uart1_data.usart_tx_buffer[13] = (uint8_t)((VERSION >> 0) & 0xFF);
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[14] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[15] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 16;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_own2_version(bool sts)
{
    uint16_t getCrc;
    memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

    uart2_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart2_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart2_data.usart_tx_buffer[2] = 0x0B;
    uart2_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 8) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_VERSION_CMD_ID >> 0) & 0xFF); // 0x02;
    uart2_data.usart_tx_buffer[5] = 0x00;
    uart2_data.usart_tx_buffer[6] = sts;
    if (sts == true)
    {
        uart2_data.usart_tx_buffer[7] = (uint8_t)((VERSION >> 24) & 0xFF);
        uart2_data.usart_tx_buffer[8] = 0x2E;
        uart2_data.usart_tx_buffer[9] = (uint8_t)((VERSION >> 16) & 0xFF);
        uart2_data.usart_tx_buffer[10] = 0x2E;
        uart2_data.usart_tx_buffer[11] = (uint8_t)((VERSION >> 8) & 0xFF);
        uart2_data.usart_tx_buffer[12] = 0x2E;
        uart2_data.usart_tx_buffer[13] = (uint8_t)((VERSION >> 0) & 0xFF);
    }
    else
    {
        uart2_data.usart_tx_buffer[7] = 0x00;
        uart2_data.usart_tx_buffer[8] = 0x00;
        uart2_data.usart_tx_buffer[9] = 0x00;
        uart2_data.usart_tx_buffer[10] = 0x00;
        uart2_data.usart_tx_buffer[11] = 0x00;
        uart2_data.usart_tx_buffer[12] = 0x00;
        uart2_data.usart_tx_buffer[13] = 0x00;
    }

    getCrc = crc16_modbus(&uart2_data.usart_tx_buffer[3], uart2_data.usart_tx_buffer[2]);
    uart2_data.usart_tx_buffer[14] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart2_data.usart_tx_buffer[15] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart2_data.usart_tx_buffer_size = 16;

    while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
    }

    uart2_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_leds_fb(uint8_t sts)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x07;
    uart1_data.usart_tx_buffer[3] = sts;
    uart1_data.usart_tx_buffer[4] = fb_ctrl_buf[4];
    uart1_data.usart_tx_buffer[5] = fb_ctrl_buf[5];
    uart1_data.usart_tx_buffer[6] = fb_ctrl_buf[6];
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[11] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 12;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_diy_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_SET_DIY_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_SET_DIY_CMD_ID >> 0) & 0xFF); // 0x03;
    uart1_data.usart_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.ctrl_diy_leds_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_diy2_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_SET_DIY2_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_SET_DIY2_CMD_ID >> 0) & 0xFF); // 0x03;
    uart1_data.usart_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.ctrl_diy2_leds_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_tray_leds_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_SET_TRAY_LEDS_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_SET_TRAY_LEDS_CMD_ID >> 0) & 0xFF); // 0x03;
    uart1_data.usart_tx_buffer[5] = (index == CTRL_OWNER_FLAG) ? 0x00 : index;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.ctrl_tray_leds;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_reset_sensor_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_RESET_SENSOR_CMD_ID >> 0) & 0xFF); // 0x04;
    uart1_data.usart_tx_buffer[5] = index;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.reset_sensor_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_usbhub_fb(uint8_t sts)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x07;
    uart1_data.usart_tx_buffer[3] = sts;
    uart1_data.usart_tx_buffer[4] = fb_ctrl_buf[4];
    uart1_data.usart_tx_buffer[5] = fb_ctrl_buf[5];
    uart1_data.usart_tx_buffer[6] = sts;
    uart1_data.usart_tx_buffer[7] = fb_ctrl_buf[7];
    uart1_data.usart_tx_buffer[8] = fb_ctrl_buf[8];
    uart1_data.usart_tx_buffer[9] = fb_ctrl_buf[9];
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[11] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 12;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_switch_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_RESET_SWITCH_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart_tx_buffer[5] = 0x00;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.reset_switch_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_andriod_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_RESET_ANDRIOD_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart_tx_buffer[5] = 0x00;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.reset_andriod_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_acc_fb(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x04;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 8) & 0xFF); // 0x02;
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_RESET_ACC_CMD_ID >> 0) & 0xFF); // 0x06;
    uart1_data.usart_tx_buffer[5] = 0x00;
    uart1_data.usart_tx_buffer[6] = uart1SendTypeFlag.reset_acc_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 9;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_ctrl_acc_old_fb(uint8_t sts)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x07;
    uart1_data.usart_tx_buffer[3] = 0x01;
    uart1_data.usart_tx_buffer[4] = fb_ctrl_buf[4];
    uart1_data.usart_tx_buffer[5] = fb_ctrl_buf[5];
    uart1_data.usart_tx_buffer[6] = fb_ctrl_buf[6];
    uart1_data.usart_tx_buffer[7] = sts;
    uart1_data.usart_tx_buffer[8] = fb_ctrl_buf[8];
    uart1_data.usart_tx_buffer[9] = fb_ctrl_buf[9];
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[10] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[11] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 12;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void send_uid(uint8_t index)
{
    uint16_t getCrc;
    memset(uart1_data.usart_tx_buffer, 0, sizeof(uart1_data.usart_tx_buffer));

    uart1_data.usart_tx_buffer[0] = UART_MSG_HEADER_1;
    uart1_data.usart_tx_buffer[1] = UART_MSG_HEADER_2;
    uart1_data.usart_tx_buffer[2] = 0x10;
    uart1_data.usart_tx_buffer[3] = (uint8_t)((FB_GET_UID_CMD_ID >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[4] = (uint8_t)((FB_GET_UID_CMD_ID >> 0) & 0xFF);
    uart1_data.usart_tx_buffer[5] = index;
    uart1_data.usart_tx_buffer[6] = uidBuf[index][11];
    uart1_data.usart_tx_buffer[7] = uidBuf[index][10];
    uart1_data.usart_tx_buffer[8] = uidBuf[index][9];
    uart1_data.usart_tx_buffer[9] = uidBuf[index][8];
    uart1_data.usart_tx_buffer[10] = uidBuf[index][7];
    uart1_data.usart_tx_buffer[11] = uidBuf[index][6];
    uart1_data.usart_tx_buffer[12] = uidBuf[index][5];
    uart1_data.usart_tx_buffer[13] = uidBuf[index][4];
    uart1_data.usart_tx_buffer[14] = uidBuf[index][3];
    uart1_data.usart_tx_buffer[15] = uidBuf[index][2];
    uart1_data.usart_tx_buffer[16] = uidBuf[index][1];
    uart1_data.usart_tx_buffer[17] = uidBuf[index][0];
    uart1_data.usart_tx_buffer[18] = uart1SendTypeFlag.uid_own_valid;
    getCrc = crc16_modbus(&uart1_data.usart_tx_buffer[3], uart1_data.usart_tx_buffer[2]);
    uart1_data.usart_tx_buffer[19] = (uint8_t)((getCrc >> 8) & 0xFF);
    uart1_data.usart_tx_buffer[20] = (uint8_t)((getCrc >> 0) & 0xFF);
    uart1_data.usart_tx_buffer_size = 21;

    while (uart1_data.usart_tx_counter < uart1_data.usart_tx_buffer_size)
    {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
            ;

        usart_data_transmit(USART1, uart1_data.usart_tx_buffer[uart1_data.usart_tx_counter++]);
    }

    uart1_data.usart_tx_counter = 0;
    vTaskDelay(5);
}

void check_version(void)
{
    send_own_version_old();
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
        if (updateFWFlag == 0)
        {
            send_sts_loop();
        }

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

void target_reset_can_data(uint8_t index)
{
    canResetBoard[index].data[0] = 0x01;
    canResetBoard[index].data[1] = 0x00;
    canResetBoard[index].data[2] = 0x00;
    canResetBoard[index].data[3] = 0x00;
    canResetBoard[index].data[4] = 0x00;
    canResetBoard[index].data[5] = 0x00;
    canResetBoard[index].data[6] = 0x00;
}
void target_set_ws2812b_can_data(uint8_t index, uint8_t mode, uint8_t r, uint8_t g, uint8_t b)
{
    canSetLeds[index].data[0] = mode;
    canSetLeds[index].data[1] = r;
    canSetLeds[index].data[2] = g;
    canSetLeds[index].data[3] = b;
    canSetLeds[index].data[4] = 0x00;
    canSetLeds[index].data[5] = 0x00;
    canSetLeds[index].data[6] = 0x00;
}

void target_reset_sensor_can_data(uint8_t index)
{
    canResetSensor[index].data[0] = 0x01;
    canResetSensor[index].data[1] = 0x00;
    canResetSensor[index].data[2] = 0x00;
    canResetSensor[index].data[3] = 0x00;
    canResetSensor[index].data[4] = 0x00;
    canResetSensor[index].data[5] = 0x00;
    canResetSensor[index].data[6] = 0x00;
}

void target_sync_fw_can_data(uint8_t index, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8)
{
    canSyncFw[index].data[0] = data1;
    canSyncFw[index].data[1] = data2;
    canSyncFw[index].data[2] = data3;
    canSyncFw[index].data[3] = data4;
    canSyncFw[index].data[4] = data5;
    canSyncFw[index].data[5] = data6;
    canSyncFw[index].data[6] = data7;
    canSyncFw[index].data[7] = data8;
}

void target_ver_uid_can_data(void)
{
    ctrlData2Can.data[0] = 0x01;
    ctrlData2Can.data[1] = 0x00;
    ctrlData2Can.data[2] = 0x00;
    ctrlData2Can.data[3] = 0x00;
    ctrlData2Can.data[4] = 0x00;
    ctrlData2Can.data[5] = 0x00;
    ctrlData2Can.data[6] = 0x00;
}

void parse_reboot_cmd(uint8_t index)
{
    if (index > TRAY_SUM)
    {
        uart1SendTypeFlag.need_reboot_valid = 0;
        uart1SendTypeFlag.need_reboot = index == 0x00 ? CTRL_OWNER_FLAG : index;
    }
    else if (index == TRAY_MASTER)
    {
        uart1SendTypeFlag.need_reboot_valid = 1;
        uart1SendTypeFlag.need_reboot = CTRL_OWNER_FLAG;
    }

    else if (index == TRAY_F1)
    {
        canResetBoard[0].standard_id = SET_F1_REBOOT_ID;
        target_reset_can_data(0);
        sendCanStr.resetBoard[0] = 1;
    }

    else if (index == TRAY_F2)
    {
        canResetBoard[1].standard_id = SET_F2_REBOOT_ID;
        target_reset_can_data(1);
        sendCanStr.resetBoard[1] = 1;
    }

    else if (index == TRAY_F3)
    {
        canResetBoard[2].standard_id = SET_F3_REBOOT_ID;
        target_reset_can_data(2);
        sendCanStr.resetBoard[2] = 1;
    }
    else if (index == TRAY_F4)
    {
        canResetBoard[3].standard_id = SET_F4_REBOOT_ID;
        target_reset_can_data(3);
        sendCanStr.resetBoard[3] = 1;
    }
}

void parse_get_ver_cmd(uint8_t index)
{
    if (index == 0)
    {
        send_own_version_old();
    }
}

void parse_diy_led_cmd(void)
{
    if (ctrl_buff[2] != 0x0C)
    {
        diyShowDataSts = 0;
        uart1SendTypeFlag.ctrl_diy_leds_valid = 0;
        uart1SendTypeFlag.ctrl_diy_leds = 0x0f; // ctrl_buff[5];
    }
    else
    {
        if ((ctrl_buff[5] != 0x00) || (ctrl_buff[14] > LED_COLOR_TYPE_MAX))
        {
            diyShowDataSts = 0;
            uart1SendTypeFlag.ctrl_diy_leds_valid = 0;
            uart1SendTypeFlag.ctrl_diy_leds = 0x0f; // ctrl_buff[5];
        }
        else
        {
            diyShowDataSts = 1;
            uart1SendTypeFlag.ctrl_diy_leds_valid = 1;
            uart1SendTypeFlag.ctrl_diy_leds = 0x0f;
        }
        if (diyShowDataSts == 1)
        {
            diyArr[0][0] = ctrl_buff[6];
            diyArr[0][1] = ctrl_buff[7];
            diyArr[1][0] = ctrl_buff[8];
            diyArr[1][1] = ctrl_buff[9];
            diyArr[2][0] = ctrl_buff[10];
            diyArr[2][1] = ctrl_buff[11];
            diyArr[3][0] = ctrl_buff[12];
            diyArr[3][1] = ctrl_buff[13];
            diyColor = ctrl_buff[14];
            ledMode = DIY_SHOW;
        }
    }
}

void parse_diy_led_old_cmd(void)
{
    uint8_t i;
    uart1SendTypeFlag.ctrl_diy_leds_valid = 1;
    for (i = 0; i < 4; i++)
    {

        if ((ctrl_buff[7 + i * 2] + ctrl_buff[8 + i * 2] > LED_SUM) || (ctrl_buff[15] > 7))
        {
            uart1SendTypeFlag.ctrl_diy_leds_valid = 0;
            diyShowDataSts = 0;
            break;
        }
    }
    if (diyShowDataSts == 1)
    {
        diyArr[0][0] = ctrl_buff[7];
        diyArr[0][1] = ctrl_buff[8];
        diyArr[1][0] = ctrl_buff[9];
        diyArr[1][1] = ctrl_buff[10];
        diyArr[2][0] = ctrl_buff[11];
        diyArr[2][1] = ctrl_buff[12];
        diyArr[3][0] = ctrl_buff[13];
        diyArr[3][1] = ctrl_buff[14];
        diyColor = ctrl_buff[15];
        ledMode = DIY_SHOW;
    }
    send_ctrl_leds_fb(uart1SendTypeFlag.ctrl_diy_leds_valid);
}

void parse_diy2_led_cmd(void)
{
    if (ctrl_buff[2] != 0x0C)
    {
        diyShowDataSts = 0;
        uart1SendTypeFlag.ctrl_diy_leds_valid = 0;
        uart1SendTypeFlag.ctrl_diy_leds = 0x0f; // ctrl_buff[5];
    }
    else
    {
        if ((ctrl_buff[5] != 0x00) || (ctrl_buff[14] > LED_COLOR_TYPE_MAX))
        {
            diyShowDataSts = 0;
            uart1SendTypeFlag.ctrl_diy2_leds_valid = 0;
            uart1SendTypeFlag.ctrl_diy2_leds = 0x0f; // ctrl_buff[5];
        }
        else
        {
            diyShowDataSts = 1;
            uart1SendTypeFlag.ctrl_diy2_leds_valid = 1;
            uart1SendTypeFlag.ctrl_diy2_leds = 0x0f;
        }
        if (diyShowDataSts == 1)
        {
            diyArr[0][0] = ctrl_buff[6];
            diyArr[0][1] = ctrl_buff[7];
            diyArr[1][0] = ctrl_buff[8];
            diyArr[1][1] = ctrl_buff[9];
            diyArr[2][0] = ctrl_buff[10];
            diyArr[2][1] = ctrl_buff[11];
            diyArr[3][0] = ctrl_buff[12];
            diyArr[3][1] = ctrl_buff[13];
            diyColor = ctrl_buff[14];
            ledMode = DIY2_SHOW;
        }
    }
}

void parse_diy2_led_old_cmd(void)
{
    uint8_t i;
    uart1SendTypeFlag.ctrl_diy2_leds_valid = 1;
    for (i = 0; i < 4; i++)
    {
        if ((ctrl_buff[7 + i * 2] + ctrl_buff[8 + i * 2] > LED_SUM) || (ctrl_buff[15] > 7))
        {
            uart1SendTypeFlag.ctrl_diy2_leds_valid = 0;
            diyShowDataSts = 0;
            break;
        }
    }
    if (diyShowDataSts == 1)
    {
        diyArr[0][0] = ctrl_buff[7];
        diyArr[0][1] = ctrl_buff[8];
        diyArr[1][0] = ctrl_buff[9];
        diyArr[1][1] = ctrl_buff[10];
        diyArr[2][0] = ctrl_buff[11];
        diyArr[2][1] = ctrl_buff[12];
        diyArr[3][0] = ctrl_buff[13];
        diyArr[3][1] = ctrl_buff[14];
        diyColor = ctrl_buff[15];
        ledMode = DIY2_SHOW;
    }
    send_ctrl_leds_fb(uart1SendTypeFlag.ctrl_diy2_leds_valid);
}

void parse_ctrl_serv_les_cmd(void)
{
    if ((ws2812b_ctrl_buf[7] > MAX_BRIGHTNESS) || (ws2812b_ctrl_buf[8] > MAX_BRIGHTNESS) || (ws2812b_ctrl_buf[9] > MAX_BRIGHTNESS))
    {
        uart1SendTypeFlag.ctrl_leds_valid = 0;
    }
    else if ((ws2812b_ctrl_buf[6] == RAINBOW) && (ws2812b_ctrl_buf[7] > 0x64))
    {
        uart1SendTypeFlag.ctrl_leds_valid = 0;
    }
    else
    {
        if ((ws2812b_ctrl_buf[6] == RAINBOW) && (ws2812b_ctrl_buf[7] <= 0x64))
        {
            rainbow_percent = ws2812b_ctrl_buf[7];
        }
        else
        {
            color_grb.r = ws2812b_ctrl_buf[7];
            color_grb.g = ws2812b_ctrl_buf[8];
            color_grb.b = ws2812b_ctrl_buf[9];
        }
        ledMode = ws2812b_ctrl_buf[6];
        uart1SendTypeFlag.ctrl_leds_valid = 1;
    }
    send_ctrl_leds_fb(uart1SendTypeFlag.ctrl_leds_valid);
}

void parse_ctrl_ws2812_cmd(void)
{
    if (ws2812b_ctrl_buf[6] <= 4)
    {
        parse_ctrl_serv_les_cmd();
    }
    else if (ws2812b_ctrl_buf[6] == 0x05)
    {
        parse_diy_led_old_cmd();
        diyShowDataSts = 1;
    }
    else if (ws2812b_ctrl_buf[6] == 0x06)
    {
        parse_diy2_led_old_cmd();
        diyShowDataSts = 1;
    }
}

void parse_ctrl_leds_cmd(void)
{
    if ((ctrl_buff[5] > TRAY_SUM) || (ctrl_buff[6] > LED_MODE_MAX) || (ctrl_buff[7] > MAX_BRIGHTNESS) || (ctrl_buff[8] > MAX_BRIGHTNESS) || (ctrl_buff[9] > MAX_BRIGHTNESS))
    {
        uart1SendTypeFlag.ctrl_leds_valid = 0;
        uart1SendTypeFlag.ctrl_leds = ctrl_buff[5] == 0x00 ? 0x0f : ctrl_buff[5];
    }
    else if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] > 0x64))
    {
        uart1SendTypeFlag.ctrl_leds_valid = 0;
        uart1SendTypeFlag.ctrl_leds = ctrl_buff[5];
    }
    else if (ctrl_buff[5] == TRAY_MASTER)
    {
        if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] <= 0x64))
        {
            rainbow_percent = ctrl_buff[7];
        }
        else
        {
            color_grb.r = ctrl_buff[7];
            color_grb.g = ctrl_buff[8];
            color_grb.b = ctrl_buff[9];
        }
        ledMode = ctrl_buff[6];

        uart1SendTypeFlag.ctrl_leds_valid = 1;
        uart1SendTypeFlag.ctrl_leds = 0x0f;
    }

    else if (ctrl_buff[5] == TRAY_F1)
    {
        canSetLeds[0].standard_id = SET_F1_WS2812B_ID;
        target_set_ws2812b_can_data(0, ctrl_buff[6], ctrl_buff[7], ctrl_buff[8], ctrl_buff[9]);
        sendCanStr.setWs2812b[0] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F2)
    {
        canSetLeds[1].standard_id = SET_F2_WS2812B_ID;
        target_set_ws2812b_can_data(1, ctrl_buff[6], ctrl_buff[7], ctrl_buff[8], ctrl_buff[9]);
        sendCanStr.setWs2812b[1] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F3)
    {
        canSetLeds[2].standard_id = SET_F3_WS2812B_ID;
        target_set_ws2812b_can_data(2, ctrl_buff[6], ctrl_buff[7], ctrl_buff[8], ctrl_buff[9]);
        sendCanStr.setWs2812b[2] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F4)
    {
        canSetLeds[3].standard_id = SET_F4_WS2812B_ID;
        target_set_ws2812b_can_data(3, ctrl_buff[6], ctrl_buff[7], ctrl_buff[8], ctrl_buff[9]);
        sendCanStr.setWs2812b[3] = 1;
    }
}

void parse_ctrl_tray_leds_cmd(void)
{
    if ((ctrl_buff[6] > LED_MODE_MAX) || (ctrl_buff[7] > MAX_BRIGHTNESS) || (ctrl_buff[8] > MAX_BRIGHTNESS) || (ctrl_buff[9] > MAX_BRIGHTNESS))
    {
        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5] == 0x00 ? 0x0f : ctrl_buff[5];
    }
    else if ((ctrl_buff[6] == RAINBOW) && (ctrl_buff[7] > 0x64))
    {
        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5];
    }

    else if (ctrl_buff[5] == TRAY_MASTER)
    {
        uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
        uart1SendTypeFlag.ctrl_tray_leds = ctrl_buff[5];
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
}

void parse_reset_sensor_cmd(void)
{
    if ((ctrl_buff[5] > TRAY_SUM) || (ctrl_buff[5] == TRAY_MASTER))
    {
        uart1SendTypeFlag.reset_sensor_valid = 0;
        uart1SendTypeFlag.reset_sensor = ctrl_buff[5];
    }

    else if (ctrl_buff[5] == TRAY_F1)
    {
        canResetSensor[0].standard_id = RESET_F1_SENSOR_ID;
        target_reset_sensor_can_data(0);
        sendCanStr.resetSensor[0] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F2)
    {
        canResetSensor[1].standard_id = RESET_F2_SENSOR_ID;
        target_reset_sensor_can_data(1);
        sendCanStr.resetSensor[1] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F3)
    {
        canResetSensor[2].standard_id = RESET_F3_SENSOR_ID;
        target_reset_sensor_can_data(2);
        sendCanStr.resetSensor[2] = 1;
    }

    else if (ctrl_buff[5] == TRAY_F4)
    {
        canResetSensor[3].standard_id = RESET_F4_SENSOR_ID;
        target_reset_sensor_can_data(3);
        sendCanStr.resetSensor[3] = 1;
    }
}

void parse_reset_usbhub_cmd(void)
{
    if (ctrl_buff[5] != 0x00)
    {
        uart1SendTypeFlag.reset_usbhub_valid = 0;
        uart1SendTypeFlag.reset_usbhub = 0x01;
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
}

void parse_ctrl_usbhub_cmd(uint8_t ctrlType)
{
    uint32_t delatCounter;
    switch (ctrlType)
    {
    case 0x00:
    {
        usbhub_ctrl(0);
        uart1SendTypeFlag.reset_usbhub_valid = 1;
        break;
    }

    case 0x01:
    {
        usbhub_ctrl(1);
        uart1SendTypeFlag.reset_usbhub_valid = 1;
        break;
    }

    case 0x02:
    {
        usbhub_ctrl(0);
        delatCounter = 20000;
        while (delatCounter--)
        {
            __NOP();
        }

        usbhub_ctrl(1);
        uart1SendTypeFlag.reset_usbhub_valid = 1;
        break;
    }

    default:
    {
        uart1SendTypeFlag.reset_usbhub_valid = 0;
        break;
    }
    }
    send_ctrl_usbhub_fb(uart1SendTypeFlag.reset_usbhub_valid);
}

void parse_reset_sw_cmd(void)
{
    if (ctrl_buff[5] != 0x00)
    {
        uart1SendTypeFlag.reset_switch_valid = 0x00;
        uart1SendTypeFlag.reset_switch = 0x01;
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
}

void parse_reset_andriod(void)
{
    if (ctrl_buff[5] != 0x00)
    {
        uart1SendTypeFlag.reset_andriod_valid = 0x00;
        uart1SendTypeFlag.reset_andriod = 0x01;
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
}

void parse_reset_acc_cmd(void)
{
    if (ctrl_buff[5] != 0x00)
    {
        uart1SendTypeFlag.reset_acc_valid = 0x00;
        uart1SendTypeFlag.reset_acc = 0x01;
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
}

void parse_ctrl_acc_cmd(uint8_t ctrlType)
{
    uint32_t delatCounter;
    switch (ctrlType)
    {
    case 0x00:
    {
        car_acc_ctrl(0);
        uart1SendTypeFlag.reset_acc_valid = 0x01;
        break;
    }
    case 0x01:
    {
        car_acc_ctrl(1);
        uart1SendTypeFlag.reset_acc_valid = 0x01;

        break;
    }
    case 0x02:
    {
        car_acc_ctrl(0);
        delatCounter = 10000;

        while (delatCounter--)
        {
            __NOP();
        }
        car_acc_ctrl(1);
        uart1SendTypeFlag.reset_acc_valid = 0x01;
        break;
    }
    default:
    {
        uart1SendTypeFlag.reset_acc_valid = 0x00;
        break;
    }
    }
    send_ctrl_acc_old_fb(uart1SendTypeFlag.reset_acc_valid);
}

void parse_get_uid_cmd(void)
{
    if (ctrl_buff[5] > TRAY_SUM)
    {
        uart1SendTypeFlag.uid_own_valid = 0;
        uart1SendTypeFlag.uid_own = 1;
    }

    else if (ctrl_buff[5] == TRAY_MASTER)
    {
        uart1SendTypeFlag.uid_own_valid = 1;
        uart1SendTypeFlag.uid_own = 1;
    }

    else
    {
        if (ctrl_buff[5] == TRAY_F1)
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

        target_ver_uid_can_data();
        can_transmit_ctrl_data(&ctrlData2Can);
    }
}

void usart1_rx_task_function(void *pvParameters)
{
    uint8_t loopFlashCnt = 0;
    uint16_t getCrc;

    init_can_msg();

    while (1)
    {
        if (recvCmdFlag == 1)
        {
            // vTaskSuspendAll();
            getCrc = crc16_modbus(&ctrl_buff[3], ctrl_buff[2]);
            if (getCrc == (((ctrl_buff[ctrl_buff[2] + 3] << 8) | ctrl_buff[ctrl_buff[2] + 4]) & 0xFFFF))
            {

#if 0
                cmdId = ( ( ctrl_buff[3]<<8 )|ctrl_buff[4] )&0xFFFF;

                switch( cmdId )
                {
                    case REBOOT_CMD_ID:
                    {
                        parse_reboot_cmd(ctrl_buff[5]);
                        break;
                    }

                    case GET_VERSION_CMD_ID:
                    {
                        parse_get_ver_cmd(ctrl_buff[5]);
                        break;
                    }

                    case SET_DIY_CMD_ID:
                    {
                    	parse_diy_led_cmd();
                        break;
                    }

                    case SET_DIY2_CMD_ID:
                    {
                    	parse_diy2_led_cmd();
                        break;
                    }

                    case SET_LEDS_CMD_ID:
                    {
                        parse_ctrl_leds_cmd();
                        break;
                    }
#if 1
                    case SET_TRAY_LEDS_CMD_ID:
                    {
                        parse_ctrl_tray_leds_cmd();
                        break;
                    }
#endif
                    case RESET_SENSOR_CMD_ID:
                    {
                        parse_reset_sensor_cmd();
                        break;
                    }

                    case RESET_USBHUB_CMD_ID:
                    {
                        parse_reset_usbhub_cmd();
                        break;
                    }
                    case RESET_SWITCH_CMD_ID:
                    {
                        parse_reset_sw_cmd();
                        break;
                    }
                    case RESET_ANDRIOD_CMD_ID:
                    {
                        parse_reset_andriod();
                        break;
                    }
                    case RESET_ACC_CMD_ID:
                    {
                        parse_reset_acc_cmd();
                        break;
                    }
                    case GET_UID_CMD_ID:
                    {
                        parse_get_uid_cmd();
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
#endif

                // uart�Լ�
                if ((ctrl_buff[2] == 0x11) && (ctrl_buff[3] == 0x10))
                {
                    while (loopFlashCnt < 5)
                    {
                        toggle_led_stat();
                        loopFlashCnt++;
                        vTaskDelay(50);
                    }
                    loopFlashCnt = 0;
                }
                else if (ctrl_buff[4] == 0x00) // master board
                {
                    if (ctrl_buff[5] == 0x22) // version
                    {
                        memset(fb_ctrl_buf, 0, sizeof(fb_ctrl_buf));
                        memcpy(fb_ctrl_buf, ctrl_buff, sizeof(fb_ctrl_buf));
                        parse_get_ver_cmd(0);
                    }
                    else if (ctrl_buff[5] == 0x21) // acc
                    {
                        memset(fb_ctrl_buf, 0, sizeof(fb_ctrl_buf));
                        memcpy(fb_ctrl_buf, ctrl_buff, sizeof(fb_ctrl_buf));
                        parse_ctrl_acc_cmd(ctrl_buff[6]);
                    }

                    else if (ctrl_buff[5] == 0x06) // usbhub
                    {
                        memset(fb_ctrl_buf, 0, sizeof(fb_ctrl_buf));
                        memcpy(fb_ctrl_buf, ctrl_buff, sizeof(fb_ctrl_buf));
                        parse_ctrl_usbhub_cmd(ctrl_buff[6]);
                    }

                    else if (ctrl_buff[5] == 0x11) // leds
                    {
                        memset(fb_ctrl_buf, 0, sizeof(fb_ctrl_buf));
                        memcpy(fb_ctrl_buf, ctrl_buff, sizeof(fb_ctrl_buf));
                        memset(ws2812b_ctrl_buf, 0, sizeof(fb_ctrl_buf));
                        memcpy(ws2812b_ctrl_buf, ctrl_buff, sizeof(fb_ctrl_buf));
                        parse_ctrl_ws2812_cmd();
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
                else if (usart_data_receive(USART1) == UART_MSG_HEADER_1)
                {
                    revFlag = UART_GET_HEADER_1;
                }
                else
                {
                    revFlag = UART_DATA_INIT;
                }
            }

            else if (revFlag == UART_GET_HEADER_2)
            {
                uart1_data.usart_rx_buffer[0] = UART_MSG_HEADER_1;
                uart1_data.usart_rx_buffer[1] = UART_MSG_HEADER_2;
                uart1_data.usart_rx_buffer[2] = usart_data_receive(USART1);

                if (uart1_data.usart_rx_buffer[2] <= UART_MSG_DATA_LEN_MAX)
                {
                    uart1_data.usart_rx_counter = 3;
                    revFlag = UART_GET_MSG_DATA_LEN;
                }
                else
                {
                    revFlag = UART_DATA_INIT;
                }
            }

            else if (revFlag == UART_GET_MSG_DATA_LEN)
            {
                uart1_data.usart_rx_buffer[uart1_data.usart_rx_counter++] = usart_data_receive(USART1);

                if (uart1_data.usart_rx_counter == uart1_data.usart_rx_buffer[2] + 5)
                {
                    memcpy(ctrl_buff, uart1_data.usart_rx_buffer, uart1_data.usart_rx_counter);
                    recvCmdFlag = 1;
                    memset(uart1_data.usart_rx_buffer, 0, sizeof(uart1_data.usart_rx_buffer));
                    uart1_data.usart_rx_counter = 0;
                    uart1_data.usart_rx_buffer_size = 0;
                    revFlag = UART_DATA_INIT;
                }
            }
        }
    }

    if (usart_flag_get(USART1, USART_TDC_FLAG) != RESET)
    {
        // USART_TDC_FLAG;
        usart_flag_clear(USART1, USART_TDC_FLAG);
    }
    if (usart_flag_get(USART1, USART_TDBE_FLAG) != RESET)
    {
        // USART_TDC_FLAG;
        usart_flag_clear(USART1, USART_TDBE_FLAG);
    }
}

uint8_t packageUpdateFwMsg(uint16_t cmd, uint8_t sts, uint8_t updateAppFlag)
{
    uint16_t getCrc;
    updateFwData[0] = 0x55;
    updateFwData[1] = 0xAA;
    updateFwData[2] = 0x04;
    updateFwData[3] = (uint8_t)((cmd >> 8) & 0xFF);
    updateFwData[4] = (uint8_t)((cmd >> 0) & 0xFF);
    updateFwData[5] = sts;
    updateFwData[6] = updateAppFlag;
    getCrc = crc16_modbus(&updateFwData[3], updateFwData[2]);
    updateFwData[7] = (uint8_t)((getCrc >> 8) & 0xFF);
    updateFwData[8] = (uint8_t)((getCrc >> 0) & 0xFF);
    return 9;
}
uint8_t packageUpdateFwMsg1(uint16_t cmd, uint8_t sts, uint16_t index)
{
    uint16_t getCrc;
    updateFwData[0] = 0x55;
    updateFwData[1] = 0xAA;
    updateFwData[2] = 0x05;
    updateFwData[3] = (uint8_t)((cmd >> 8) & 0xFF);
    updateFwData[4] = (uint8_t)((cmd >> 0) & 0xFF);
    updateFwData[5] = sts;
    updateFwData[6] = (uint8_t)((index >> 8) & 0xFF);
    updateFwData[7] = (uint8_t)((index >> 0) & 0xFF);
    getCrc = crc16_modbus(&updateFwData[3], updateFwData[2]);
    updateFwData[8] = (uint8_t)((getCrc >> 8) & 0xFF);
    updateFwData[9] = (uint8_t)((getCrc >> 0) & 0xFF);
    return 10;
}

void send_2_uart2(uint8_t *pdate, uint8_t len)
{
    uint8_t i;

    while (sendingUart2 == 1)
    {
        vTaskDelay(2);
    }
    sendingUart2 = 1;

    if (len < 128)
    {
        memset(uart2_data.usart_tx_buffer, 0, sizeof(uart2_data.usart_tx_buffer));

        for (i = 0; i < len; i++)
        {
            uart2_data.usart_tx_buffer[i] = pdate[i];
        }

        uart2_data.usart_tx_buffer_size = len;

        while (uart2_data.usart_tx_counter < uart2_data.usart_tx_buffer_size)
        {
            while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET)
                ;

            usart_data_transmit(USART2, uart2_data.usart_tx_buffer[uart2_data.usart_tx_counter++]);
        }

        uart2_data.usart_tx_counter = 0;
        vTaskDelay(5);
        sendingUart2 = 0;
    }
}

void parse_get_app_flag_msg(void)
{
    if ((updateFw.updateFwFlag == UPDATE_NONE) || (updateFw.updateFwFlag == UPDATE_FW_INFO))
    {
        // update app addr
        if ((ctrl_buff2[5] == 1) && (ctrl_buff2[6] == 1))
        {
            msglen2 = packageUpdateFwMsg(FB_GET_UPDATE_APP_FLAG_ID, VALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
            updateFw.updateFwFlag = UPDATE_FW_INFO;
        }

        else
        {
            msglen2 = packageUpdateFwMsg(FB_GET_UPDATE_APP_FLAG_ID, INVALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
        }
    }

    else
    {
        msglen2 = packageUpdateFwMsg(FB_GET_UPDATE_APP_FLAG_ID, INVALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
    }
}

void parse_send_fw_info_msg(void)
{
    if (updateFw.updateFwFlag == UPDATE_FW_INFO)
    {
        if (ctrl_buff2[6] != updateFw.updateAppAddrFlag)
        {
            updateFw.updateFwFlag = UPDATE_NONE;
            msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_FW_INFO_ID, INVALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
        }

        else
        {
            updateFw.fileSize = ((ctrl_buff2[7] << 24) | (ctrl_buff2[8] << 16) | (ctrl_buff2[9] << 8) | (ctrl_buff2[10] << 0)) & 0xFFFFFFFF;
            updateFw.fwMsgSum = ((ctrl_buff2[11] << 8) | (ctrl_buff2[12] << 0)) & 0xFFFF;

            if ((updateFw.fileSize % 128) == 0)
            {
                updateFw.fwMsgSumConform = (uint16_t)(updateFw.fileSize / 128);
            }

            else
            {
                updateFw.fwMsgSumConform = (uint16_t)(updateFw.fileSize / 128 + 1);
            }

            if ((updateFw.fileSize > APP_FW_MAX_LEN) || (updateFw.fwMsgSum != updateFw.fwMsgSumConform))
            {
                updateFw.updateFwFlag = UPDATE_NONE;
                msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_FW_INFO_ID, INVALID, updateFw.updateAppAddrFlag);
                send_2_uart2(updateFwData, msglen2);
            }

            else
            {
                msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_FW_INFO_ID, VALID, updateFw.updateAppAddrFlag);
                send_2_uart2(updateFwData, msglen2);
                updateFw.updateFwFlag = UPDATE_FW_START;
            }
        }
    }

    else
    {
        updateFw.updateFwFlag = UPDATE_NONE;
        msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_FW_INFO_ID, INVALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
    }
}

void parse_send_start_update_signal(void)
{
    if (updateFw.updateFwFlag == UPDATE_FW_START)
    {
        if ((ctrl_buff2[5] != 0x01) || (ctrl_buff2[6] != updateFw.updateAppAddrFlag))
        {
            updateFw.updateFwFlag = UPDATE_NONE;
            msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_START_ID, INVALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
        }

        else
        {
            updateFw.updateAppAddr = updateFw.updateAppBaseAddr;
            xorData[0] = 0xFF;
            xorData[1] = 0xFF;
            updateFw.fileSizeConform = ((ctrl_buff2[7] << 24) | (ctrl_buff2[8] << 16) | (ctrl_buff2[9] << 8) | (ctrl_buff2[10] << 0)) & 0xFFFFFFFF;
            updateFw.fwMsgSumConform = ((ctrl_buff2[11] << 8) | (ctrl_buff2[12] << 0)) & 0xFFFF;

            if ((updateFw.fileSizeConform != updateFw.fileSize) || (updateFw.fwMsgSumConform != updateFw.fwMsgSum))
            {
                updateFw.updateFwFlag = UPDATE_NONE;
                msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_START_ID, INVALID, updateFw.updateAppAddrFlag);
                send_2_uart2(updateFwData, msglen2);
            }

            else
            {
                updateFWFlag = 1;
                // nvic_irq_disable(USART1_IRQn);
                msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_START_ID, VALID, updateFw.updateAppAddrFlag);
                send_2_uart2(updateFwData, msglen2);
                updateFw.updateFwFlag = UPDATE_FW_ING;
            }
        }
    }

    else
    {
        updateFw.updateFwFlag = UPDATE_NONE;
        msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_START_ID, INVALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
    }
}

void parse_fw_data_msg(void)
{
    updateFw.currentIndex = ((ctrl_buff2[6] << 8) | (ctrl_buff2[7] << 0)) & 0xFFFF;

    if (updateFw.updateFwFlag == UPDATE_FW_ING)
    {
        if (ctrl_buff2[5] != 0x01)
        {
            updateFw.updateFwFlag = UPDATE_NONE;
            msglen2 = packageUpdateFwMsg1(FB_SET_UPDATE_DATA_ID, INVALID, updateFw.currentIndex);
            send_2_uart2(updateFwData, msglen2);
        }

        else
        {
            if (updateFw.currentIndex >= updateFw.fwMsgSum)
            {
                msglen2 = packageUpdateFwMsg1(FB_SET_UPDATE_DATA_ID, INVALID, updateFw.currentIndex);
                send_2_uart2(updateFwData, msglen2);
                updateFw.updateFwFlag = UPDATE_NONE;
            }

            else
            {
                // get crc
                xorData[0] = crc8_rcc(xorData[0], ctrl_buff2[ctrl_buff2[2] + 3]);
                xorData[1] = crc8_rcc(xorData[1], ctrl_buff2[ctrl_buff2[2] + 4]);
                memcpy(&update_buff[updateFw.currentIndex % 16][0], &ctrl_buff2[8], ctrl_buff2[2] - 5);

                if ((updateFw.currentIndex == (updateFw.fwMsgSum - 1)) || ((updateFw.currentIndex % 16) == 15))
                {
                    // writeflash
                    write2flash(updateFw.updateAppAddr, &update_buff[0][0], sizeof(update_buff));
                    updateFw.updateAppAddr += 2048;
                    memset(update_buff, 0, sizeof(update_buff));
                }

                msglen2 = packageUpdateFwMsg1(FB_SET_UPDATE_DATA_ID, VALID, updateFw.currentIndex);
                send_2_uart2(updateFwData, msglen2);

                if (updateFw.currentIndex == updateFw.fwMsgSum - 1)
                {
                    updateFw.updateFwFlag = UPDATE_END;
                }
            }
        }
    }

    else
    {
        updateFw.updateFwFlag = UPDATE_NONE;
        msglen2 = packageUpdateFwMsg1(FB_SET_UPDATE_DATA_ID, INVALID, updateFw.currentIndex);
        send_2_uart2(updateFwData, msglen2);
    }
}

void parse_update_end_msg(void)
{
    if (updateFw.updateFwFlag == UPDATE_END)
    {
        if (ctrl_buff2[5] != 0x01)
        {
            msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_END_ID, INVALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
        }

        else if ((ctrl_buff2[6] != xorData[0]) || (ctrl_buff2[7] != xorData[1]))
        {
            msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_END_ID, INVALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
        }

        else
        {
            updateFw.updateConformAddr[0] = updateFw.updateAppAddrFlag;
            write2flash(START_APP_FLAG_ADDR, updateFw.updateConformAddr, 1);
            msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_END_ID, VALID, updateFw.updateAppAddrFlag);
            send_2_uart2(updateFwData, msglen2);
            vTaskDelay(1000);
            nvic_system_reset();

            // xorData[0] = 0xFF;
            // xorData[1] = 0xFF;
            // updateFw.updateAppAddr = updateFw.updateAppBaseAddr;
            // updateFWFlag = 0;
            // updateFw.updateFwFlag = UPDATE_NONE;
        }
    }

    else
    {
        msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_END_ID, INVALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
    }
}

void parse_update_abort_msg(void)
{
    if ((ctrl_buff2[5] != 0x01) || (ctrl_buff2[5] != 0x01))
    {
        msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_ABORT_ID, INVALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
    }

    else
    {
        msglen2 = packageUpdateFwMsg(FB_SET_UPDATE_ABORT_ID, VALID, updateFw.updateAppAddrFlag);
        send_2_uart2(updateFwData, msglen2);
        updateFw.updateAppAddr = updateFw.updateAppBaseAddr;
        updateFw.updateFwFlag = UPDATE_NONE;
        xorData[0] = 0xFF;
        xorData[1] = 0xFF;
        updateFWFlag = 0;
        // usart_interrupt_enable( USART1, USART_RDBF_INT, TRUE );
    }
}

void parse_sync_sub_fw_msg(void)
{
    if ((ctrl_buff[5] > 0) && (ctrl_buff[5] < 5))
    {
        if (ctrl_buff[5] == TRAY_F1)
        {
            canSyncFw[0].standard_id = SYNC_F1_FW_ID;
        }
        else if (ctrl_buff[5] == TRAY_F2)
        {
            canSyncFw[1].standard_id = SYNC_F2_FW_ID;
        }
        else if (ctrl_buff[5] == TRAY_F3)
        {
            canSyncFw[2].standard_id = SYNC_F3_FW_ID;
        }
        else if (ctrl_buff[5] == TRAY_F4)
        {
            canSyncFw[3].standard_id = SYNC_F4_FW_ID;
        }
        target_sync_fw_can_data(ctrl_buff[5] - 1, ctrl_buff[6], ctrl_buff[7], ctrl_buff[8], ctrl_buff[9], ctrl_buff[10], ctrl_buff[11], ctrl_buff[12], ctrl_buff[13]);
        sendCanStr.sync_fw[ctrl_buff[5] - 1] = 1;
    }
    else
    {
        uart2SendTypeFlag.fw_info.sync_sub_fw_valid = 0;
        uart2SendTypeFlag.fw_info.sync_sub_fw = ctrl_buff[5];
    }
}

#if 1
void usart2_rx_task_function(void *pvParameters)
{
    uint16_t getCrc2;

    while (1)
    {
        if (recvCmdFlag2 == 1)
        {
            getCrc2 = crc16_modbus(&ctrl_buff2[3], ctrl_buff2[2]);
            if (getCrc2 == (((ctrl_buff2[ctrl_buff2[2] + 3] << 8) | ctrl_buff2[ctrl_buff2[2] + 4]) & 0xFFFF))
            {
                cmdId2 = ((ctrl_buff2[3] << 8) | ctrl_buff2[4]) & 0xFFFF;

                switch (cmdId2)
                {
                case REBOOT_CMD_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        send_reboot2_fb(1);
                    }
                    else
                    {
                        send_reboot2_fb(0);
                    }
                    break;
                }
                case GET_VERSION_CMD_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        send_own2_version(true);
                    }
                    else
                    {
                        send_own2_version(false);
                    }
                    break;
                }
#ifdef IHAWK_CTRL // Ӳ�������ݲ�֧��
                case SET_IHAWK_CMD_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        if ((ctrl_buff2[6] > 0) && (ctrl_buff2[6] <= 5))
                        {
                            //   ihawk_ctrl(ctrl_buff2[6], ctrl_buff2[7]);
                            if (ctrl_buff2[7] < 2)
                            {
                                send_ctrl_ihawk_fb(ctrl_buff2[6], ctrl_buff2[7], 1);
                                ihawk_ctrl(ctrl_buff2[6], ctrl_buff2[7]);
                            }
                            else if ((ctrl_buff2[7] >= 0x81) && (ctrl_buff2[7] <= 0x94))
                            {
                                send_ctrl_ihawk_fb(ctrl_buff2[6], ctrl_buff2[7], 1);
                                ihawk_ctrl_tm(ctrl_buff2[6], RESET_IHAWK, ctrl_buff2[7] & 0x7F);
                            }
                            else
                            {
                                send_ctrl_ihawk_fb(ctrl_buff2[6], ctrl_buff2[7], 0);
                            }
                        }
                        else
                        {
                            send_ctrl_ihawk_fb(ctrl_buff2[6], ctrl_buff2[7], 0);
                        }
                    }
                    else
                    {
                        send_ctrl_ihawk_fb(ctrl_buff2[6], ctrl_buff2[7], 0);
                    }
                    break;
                }
                case GET_IHAWK_STS_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        send_ihawk_sts_fb(1);
                    }
                    else
                    {
                        send_ihawk_sts_fb(0);
                    }
                    break;
                }
                case GET_USB_STS_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        send_usb_sts_fb(1);
                    }
                    else
                    {
                        send_usb_sts_fb(0);
                    }
                    break;
                }
                case RESET_ANDRIOD_CMD_ID:
                {
                    if (ctrl_buff2[5] == TRAY_MASTER)
                    {
                        if (ctrl_buff2[7] < 2)
                        {
                            send_andriod_fb(ctrl_buff2[7], 1);
                            parse_ctrl_andriod(ctrl_buff2[7]);
                        }
                        else if ((ctrl_buff2[7] >= 0x81) && (ctrl_buff2[7] <= 0x94))
                        {
                            send_andriod_fb(ctrl_buff2[7], 1);
                            parse_ctrl_andriod_tm(3, ctrl_buff2[7] & 0x7F);
                        }
                        else
                        {
                            send_andriod_fb(ctrl_buff2[7], 0);
                        }
                    }
                    else
                    {
                        send_andriod_fb(ctrl_buff2[7], 0);
                    }
                    parse_reset_andriod();

                    break;
                }
#endif
                case GET_UPDATE_APP_FLAG_ID:
                {
                    parse_get_app_flag_msg();
                    break;
                }
                case SET_UPDATE_FW_INFO_ID: // send firmware info
                {
                    parse_send_fw_info_msg();
                    break;
                }
                case SET_UPDATE_START_ID: // start update signal
                {
                    parse_send_start_update_signal();
                    break;
                }

                case SET_UPDATE_DATA_ID: // updating firmware
                {
                    parse_fw_data_msg();
                    break;
                }

                case SET_UPDATE_END_ID: // update firmware sending completed
                {
                    parse_update_end_msg();
                    break;
                }

                case SET_UPDATE_ABORT_ID: // abort update
                {
                    parse_update_abort_msg();
                    break;
                }
                    //                    case SYNC_SUB_FW_INFO_ID:  // get appaddr/sync fw info/update end/update abort
                    //                    {
                    //                        parse_sync_sub_fw_msg();
                    //                        break;
                    //                    }
                    //                    case SUB_UPDATING_APP_ID:  // updating
                    //                    {

                    //                        break;
                    //                    }

                default:
                {
                    break;
                }
                }
            }
            // xTaskResumeAll();
            recvCmdFlag2 = 0;
        }
        taskAliveBits |= TASK_UART_RX_BIT_6;
        vTaskDelay(5);
    }
}

void USART2_IRQHandler(void)
{
    static uint8_t revFlag2 = UART_DATA_INIT;

    if (usart_flag_get(USART2, USART_RDBF_FLAG) != RESET)
    {
        usart_flag_clear(USART2, USART_RDBF_FLAG);
        if (recvCmdFlag2 == 0)
        {
            if (revFlag2 == UART_DATA_INIT)
            {
                if (usart_data_receive(USART2) == UART_MSG_HEADER_1)
                {
                    revFlag2 = UART_GET_HEADER_1;
                }
            }

            else if (revFlag2 == UART_GET_HEADER_1)
            {
                if (usart_data_receive(USART2) == UART_MSG_HEADER_2)
                {
                    revFlag2 = UART_GET_HEADER_2;
                }
                else if (usart_data_receive(USART2) == UART_MSG_HEADER_1)
                {
                    revFlag2 = UART_GET_HEADER_1;
                }
                else
                {
                    revFlag2 = UART_DATA_INIT;
                }
            }

            else if (revFlag2 == UART_GET_HEADER_2)
            {
                uart2_data.usart_rx_buffer[0] = UART_MSG_HEADER_1;
                uart2_data.usart_rx_buffer[1] = UART_MSG_HEADER_2;
                uart2_data.usart_rx_buffer[2] = usart_data_receive(USART2);

                if (uart2_data.usart_rx_buffer[2] <= UART_MSG_DATA_LEN_MAX)
                {
                    uart2_data.usart_rx_counter = 3;
                    revFlag2 = UART_GET_MSG_DATA_LEN;
                }
                else
                {
                    revFlag2 = UART_DATA_INIT;
                }
            }

            else if (revFlag2 == UART_GET_MSG_DATA_LEN)
            {
                uart2_data.usart_rx_buffer[uart2_data.usart_rx_counter++] = usart_data_receive(USART2);

                if (uart2_data.usart_rx_counter == uart2_data.usart_rx_buffer[2] + 5)
                {
                    memcpy(ctrl_buff2, uart2_data.usart_rx_buffer, uart2_data.usart_rx_counter);
                    recvCmdFlag2 = 1;
                    memset(uart2_data.usart_rx_buffer, 0, sizeof(uart2_data.usart_rx_buffer));
                    uart2_data.usart_rx_counter = 0;
                    uart2_data.usart_rx_buffer_size = 0;
                    revFlag2 = UART_DATA_INIT;
                }
            }
        }
    }

    if (usart_flag_get(USART2, USART_TDBE_FLAG) != RESET)
    {
        // USART_TDC_FLAG;
    }
}
#endif
