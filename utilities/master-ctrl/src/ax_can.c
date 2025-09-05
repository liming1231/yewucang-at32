#include <string.h>
#include "ax_can.h"
#include "ax_iic.h"
#include "ax_ws2812.h"

volatile uint8_t newMsgFlag = 0;

can_rx_message_type rx_message_struct_g;

struct can_alive_counter canAliveCounter = {0};

void canbus_sw_config(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CAN_BUS_SHON_CLOCK, TRUE);
    crm_periph_clock_enable(CAN_BUS_RS_CLOCK, TRUE);

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = CAN_BUS_SHON_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_init(CAN_BUS_SHON_PORT, &gpio_init_struct);

    gpio_init_struct.gpio_pins = CAN_BUS_RS_PIN;
    gpio_init(CAN_BUS_RS_PORT, &gpio_init_struct);
}

void canbus_open(void)
{
    canbus_sw_config();
    gpio_bits_reset(CAN_BUS_SHON_PORT, CAN_BUS_SHON_PIN);
    gpio_bits_reset(CAN_BUS_RS_PORT, CAN_BUS_RS_PIN);
}

void can_alive_struct_init(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        canAliveCounter.canLastCounter[i] = 0;
        canAliveCounter.canNowCounter[i] = 0;
        canAliveCounter.canTimeoutCounter[i] = 5;
        canAliveCounter.onLine[i] = 0;
    }
    canAliveCounter.loopCounter = 0;
}

/**
 *  @brief  can gpio config
 *  @param  none
 *  @retval none
 */
void can_gpio_config(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(CAN1_GMUX_0010, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    /* can tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    /* can rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);
}

/**
 *  @brief  can configiguration.
 *  @param  none
 *  @retval none
 */
void can_configuration(void)
{
    can_base_type can_base_struct;
    can_baudrate_type can_baudrate_struct;
    can_filter_init_type can_filter_init_struct;

    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    /* can base init */
    can_default_para_init(&can_base_struct);
    can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
    can_base_struct.ttc_enable = FALSE;
    can_base_struct.aebo_enable = TRUE;
    can_base_struct.aed_enable = TRUE;
    can_base_struct.prsf_enable = FALSE;
    can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
    can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;
    can_base_init(CAN1, &can_base_struct);

    /* can baudrate, set baudrate = pclk/(baudrate_div *(1 + bts1_size + bts2_size)) */
    can_baudrate_struct.baudrate_div = 6;
    can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
    can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
    can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
    can_baudrate_set(CAN1, &can_baudrate_struct);

    /* can filter init */
    can_filter_init_struct.filter_activate_enable = TRUE;
    can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;
    can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
    can_filter_init_struct.filter_number = 0;
    can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
    can_filter_init_struct.filter_id_high = 0;
    can_filter_init_struct.filter_id_low = 0;
    can_filter_init_struct.filter_mask_high = 0;
    can_filter_init_struct.filter_mask_low = 0;
    can_filter_init(CAN1, &can_filter_init_struct);

    /* can interrupt config */
    nvic_irq_enable(CAN1_SE_IRQn, 0x00, 0x00);
    nvic_irq_enable(CAN1_RX0_IRQn, 0x00, 0x00);
    can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);

    /* error interrupt enable */
    can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);
}

uint8_t getSumCrc(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t sum = 0;

    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFF);
}

void can_transmit_ctrl_data(can_tx_message_type *tx_message_struct)
{
#ifdef CAN_DAUL
    static uint16_t count1 = 0;
    uint8_t transmit_mailbox;

    while (can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) == CAN_TX_STATUS_NO_EMPTY)
        ;

    tx_message_struct->data[7] = getSumCrc(tx_message_struct->data, 7);
    transmit_mailbox = can_message_transmit(CAN1, tx_message_struct);

    while (can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) != CAN_TX_STATUS_SUCCESSFUL)
    {
        count1++;
        if (count1 == 0xFFFF)
        {
            break;
        }
    }
    count1 = 0;
#else
    ;
#endif
}

void parse_reboot_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_F1_SET_REBOOT_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            uart1SendTypeFlag.need_reboot_valid = 1;
        }
        else
        {
            uart1SendTypeFlag.need_reboot_valid = 0;
        }
        uart1SendTypeFlag.need_reboot = 1;
        break;
    }
    case FB_F2_SET_REBOOT_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            uart1SendTypeFlag.need_reboot_valid = 1;
        }
        else
        {
            uart1SendTypeFlag.need_reboot_valid = 0;
        }
        uart1SendTypeFlag.need_reboot = 2;
        break;
    }
    case FB_F3_SET_REBOOT_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            uart1SendTypeFlag.need_reboot_valid = 1;
        }
        else
        {
            uart1SendTypeFlag.need_reboot_valid = 0;
        }
        uart1SendTypeFlag.need_reboot = 3;
        break;
    }
    case FB_F4_SET_REBOOT_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            uart1SendTypeFlag.need_reboot_valid = 1;
        }
        else
        {
            uart1SendTypeFlag.need_reboot_valid = 0;
        }
        uart1SendTypeFlag.need_reboot = 4;
        break;
    }

    default:
        break;
    }
}

void parse_heartbeat_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_F1_HEARDBEAT_ID:
    {
        if (rx_message_struct_g.data[0] == 0x01)
        {
            // online
        }

        break;
    }

    case FB_F2_HEARDBEAT_ID:
    {
        if (rx_message_struct_g.data[0] == 0x01)
        {
            // online
        }

        break;
    }

    case FB_F3_HEARDBEAT_ID:
    {
        if (rx_message_struct_g.data[0] == 0x01)
        {
            // online
        }

        break;
    }

    case FB_F4_HEARDBEAT_ID:
    {
        if (rx_message_struct_g.data[0] == 0x01)
        {
            // online
        }

        break;
    }

    default:
        break;
    }
}

void parse_version_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_GET_F1_VERSION_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            memcpy(versionSub[0], rx_message_struct_g.data, 6);
            uart1SendTypeFlag.version_f1 = 1;
        }

        break;
    }

    case FB_GET_F2_VERSION_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            memcpy(versionSub[1], rx_message_struct_g.data, 6);
            uart1SendTypeFlag.version_f2 = 1;
        }

        break;
    }

    case FB_GET_F3_VERSION_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            memcpy(versionSub[2], rx_message_struct_g.data, 6);
            uart1SendTypeFlag.version_f3 = 1;
        }

        break;
    }

    case FB_GET_F4_VERSION_ID:
    {
        if (rx_message_struct_g.data[6] == 0x01)
        {
            memcpy(versionSub[3], rx_message_struct_g.data, 6);
            uart1SendTypeFlag.version_f4 = 1;
        }

        break;
    }

    default:
        break;
    }
}

void parse_uid_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_GET_F1_UID_ID:
    {
        if (rx_message_struct_g.data[7] == 0x01)
        {
            if (rx_message_struct_g.data[6] == 0x01)
            {
                memcpy(&uidBuf[1][0], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f1 |= 0x01;
            }
            else if (rx_message_struct_g.data[6] == 0x02)
            {
                memcpy(&uidBuf[1][6], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f1 |= 0x02;
            }
        }

        break;
    }

    case FB_GET_F2_UID_ID:
    {
        if (rx_message_struct_g.data[7] == 0x01)
        {
            if (rx_message_struct_g.data[6] == 0x01)
            {
                memcpy(&uidBuf[2][0], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f2 |= 1;
            }
            else if (rx_message_struct_g.data[6] == 0x02)
            {
                memcpy(&uidBuf[2][6], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f2 |= 2;
            }
        }

        break;
    }

    case FB_GET_F3_UID_ID:
    {
        if (rx_message_struct_g.data[7] == 0x01)
        {
            if (rx_message_struct_g.data[6] == 0x01)
            {
                memcpy(&uidBuf[3][0], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f3 |= 1;
            }
            else if (rx_message_struct_g.data[6] == 0x02)
            {
                memcpy(&uidBuf[3][6], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f3 |= 2;
            }
        }

        break;
    }

    case FB_GET_F4_UID_ID:
    {
        if (rx_message_struct_g.data[7] == 0x01)
        {
            if (rx_message_struct_g.data[6] == 0x01)
            {
                memcpy(&uidBuf[4][0], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f4 |= 1;
            }
            else if (rx_message_struct_g.data[6] == 0x02)
            {
                memcpy(&uidBuf[4][6], rx_message_struct_g.data, 6);
                uart1SendTypeFlag.uid_f4 |= 2;
            }
        }

        break;
    }

    default:
        break;
    }
}

void parse_set_ws2812_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_SET_F1_WS2812B_ID:
    {
        uart1SendTypeFlag.ctrl_leds_valid = rx_message_struct_g.data[6];
        uart1SendTypeFlag.ctrl_leds = 1;
        break;
    }

    case FB_SET_F2_WS2812B_ID:
    {
        uart1SendTypeFlag.ctrl_leds_valid = rx_message_struct_g.data[6];
        uart1SendTypeFlag.ctrl_leds = 2;
        break;
    }

    case FB_SET_F3_WS2812B_ID:
    {
        uart1SendTypeFlag.ctrl_leds_valid = rx_message_struct_g.data[6];
        uart1SendTypeFlag.ctrl_leds = 3;
        break;
    }

    case FB_SET_F4_WS2812B_ID:
    {
        uart1SendTypeFlag.ctrl_leds_valid = rx_message_struct_g.data[6];
        uart1SendTypeFlag.ctrl_leds = 4;
        break;
    }

    default:
        break;
    }
}

void parse_set_tray_ws2812_fb_msg(void)
{
    static uint8_t fb_tray_ws2812b_sts = 0;
    static uint8_t tray_ws2812b_cmd_valid = 0;
    switch (rx_message_struct_g.standard_id)
    {
    case FB_SET_TRAY_F1_WS2812B_ID:
    {
        fb_tray_ws2812b_sts |= (1 << 0);
        tray_ws2812b_cmd_valid |= (uint8_t)((rx_message_struct_g.data[6] << 0) & 0xFF);
        break;
    }
    case FB_SET_TRAY_F2_WS2812B_ID:
    {
        fb_tray_ws2812b_sts |= (1 << 1);
        tray_ws2812b_cmd_valid |= (uint8_t)((rx_message_struct_g.data[6] << 1) & 0xFF);
        break;
    }
    case FB_SET_TRAY_F3_WS2812B_ID:
    {
        fb_tray_ws2812b_sts |= (1 << 2);
        tray_ws2812b_cmd_valid |= (uint8_t)((rx_message_struct_g.data[6] << 2) & 0xFF);
        break;
    }
    case FB_SET_TRAY_F4_WS2812B_ID:
    {
        fb_tray_ws2812b_sts |= (1 << 3);
        tray_ws2812b_cmd_valid |= (uint8_t)((rx_message_struct_g.data[6] << 3) & 0xFF);
        break;
    }
    default:
    {
        break;
    }
    }

    if (fb_tray_ws2812b_sts == rx_message_struct_g.data[4])
    {
        if (tray_ws2812b_cmd_valid == rx_message_struct_g.data[4])
        {
            uart1SendTypeFlag.ctrl_tray_leds_valid = 1;
        }
        else
        {
            uart1SendTypeFlag.ctrl_tray_leds_valid = 0;
        }
        uart1SendTypeFlag.ctrl_tray_leds = fb_tray_ws2812b_sts;
        fb_tray_ws2812b_sts = 0;
        tray_ws2812b_cmd_valid = 0;
    }
}

void parse_reset_sensor_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_RESET_F1_SENSOR_ID:
    {
        uart1SendTypeFlag.reset_sensor_valid = 1;
        uart1SendTypeFlag.reset_sensor = 1;
        break;
    }

    case FB_RESET_F2_SENSOR_ID:
    {
        uart1SendTypeFlag.reset_sensor_valid = 1;
        uart1SendTypeFlag.reset_sensor = 2;
        break;
    }

    case FB_RESET_F3_SENSOR_ID:
    {
        uart1SendTypeFlag.reset_sensor_valid = 1;
        uart1SendTypeFlag.reset_sensor = 3;
        break;
    }

    case FB_RESET_F4_SENSOR_ID:
    {
        uart1SendTypeFlag.reset_sensor_valid = 1;
        uart1SendTypeFlag.reset_sensor = 4;
        break;
    }

    default:
        break;
    }
}

void parse_sync_fw_fb_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case FB_SYNC_F1_FW_ID:
    {
        if (rx_message_struct_g.data[0] == 1)
        {
            uart2SendTypeFlag.fw_info.sync_sub_fw = 1;
            if ((rx_message_struct_g.data[1] == 1) && ((rx_message_struct_g.data[2] == 1) || (rx_message_struct_g.data[2] == 2)))
            {
                uart2SendTypeFlag.fw_info.update_addr = rx_message_struct_g.data[1];
                uart2SendTypeFlag.fw_info.sync_sub_fw_valid = 1;
            }
            else
            {
                uart2SendTypeFlag.fw_info.sync_sub_fw_valid = 0;
            }
        }
        else if (rx_message_struct_g.data[0] == 2)
        {
            uart2SendTypeFlag.fw_info.sync_sub_fw = 1;
            if ((rx_message_struct_g.data[1] == 1) && (uart2SendTypeFlag.fw_info.update_addr == rx_message_struct_g.data[2]))
            {
                uart2SendTypeFlag.fw_info.sync_sub_fw_valid = 1;
                uart2SendTypeFlag.fw_info.fw_length = ((rx_message_struct_g.data[3] << 8) | (rx_message_struct_g.data[4])) & 0xFFFF;
                uart2SendTypeFlag.fw_info.fw_msg_counter = ((rx_message_struct_g.data[5] << 8) | (rx_message_struct_g.data[6])) & 0xFFFF;
                uart2SendTypeFlag.fw_info.fw_index = 0;
            }
            else
            {
                uart2SendTypeFlag.fw_info.sync_sub_fw_valid = 0;
            }
            uart2SendTypeFlag.fw_info.update_addr = 0;
        }

        break;
    }

    case FB_SYNC_F2_FW_ID:
    {
        break;
    }

    case FB_SYNC_F3_FW_ID:
    {
        break;
    }

    case FB_SYNC_F4_FW_ID:
    {
        break;
    }

    default:
        break;
    }
}

void parse_distance_msg()
{
    switch (rx_message_struct_g.standard_id)
    {
    case RECV_F1_DISTANCE_ID:
    {
        distance[0][0] = rx_message_struct_g.data[0];
        distance[0][1] = rx_message_struct_g.data[1];
        distance[0][2] = rx_message_struct_g.data[2];
        canAliveCounter.canNowCounter[0] = rx_message_struct_g.data[6];
        break;
    }

    case RECV_F2_DISTANCE_ID:
    {
        distance[1][0] = rx_message_struct_g.data[0];
        distance[1][1] = rx_message_struct_g.data[1];
        distance[1][2] = rx_message_struct_g.data[2];
        canAliveCounter.canNowCounter[1] = rx_message_struct_g.data[6];
        break;
    }

    case RECV_F3_DISTANCE_ID:
    {
        distance[2][0] = rx_message_struct_g.data[0];
        distance[2][1] = rx_message_struct_g.data[1];
        distance[2][2] = rx_message_struct_g.data[2];
        canAliveCounter.canNowCounter[2] = rx_message_struct_g.data[6];
        break;
    }

    case RECV_F4_DISTANCE_ID:
    {
        distance[3][0] = rx_message_struct_g.data[0];
        distance[3][1] = rx_message_struct_g.data[1];
        distance[3][2] = rx_message_struct_g.data[2];
        canAliveCounter.canNowCounter[3] = rx_message_struct_g.data[6];
        break;
    }

    default:
    {
        break;
    }
    }
}

void send_ws2812b_msg(void)
{
    if (sendCanStr.setTrayWs2812b == 1)
    {
        can_transmit_ctrl_data(&canSetTrayLeds);
        sendCanStr.setTrayWs2812b = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }
    else
    {

        if (sendCanStr.setWs2812b[0] == 1)
        {
            can_transmit_ctrl_data(&canSetLeds[0]);
            sendCanStr.setWs2812b[0] = 0;
            vTaskDelay(SEND_CAN_MSG_TMVAL);
        }

        if (sendCanStr.setWs2812b[1] == 1)
        {
            can_transmit_ctrl_data(&canSetLeds[1]);
            sendCanStr.setWs2812b[1] = 0;
            vTaskDelay(SEND_CAN_MSG_TMVAL);
        }

        if (sendCanStr.setWs2812b[2] == 1)
        {
            can_transmit_ctrl_data(&canSetLeds[2]);
            sendCanStr.setWs2812b[2] = 0;
            vTaskDelay(SEND_CAN_MSG_TMVAL);
        }

        if (sendCanStr.setWs2812b[3] == 1)
        {
            can_transmit_ctrl_data(&canSetLeds[3]);
            sendCanStr.setWs2812b[3] = 0;
            vTaskDelay(SEND_CAN_MSG_TMVAL);
        }
    }
}

void send_reset_sensor_msg(void)
{
    if (sendCanStr.resetSensor[0] == 1)
    {
        can_transmit_ctrl_data(&canResetSensor[0]);
        sendCanStr.resetSensor[0] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetSensor[1] == 1)
    {
        can_transmit_ctrl_data(&canResetSensor[1]);
        sendCanStr.resetSensor[1] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetSensor[2] == 1)
    {
        can_transmit_ctrl_data(&canResetSensor[2]);
        sendCanStr.resetSensor[2] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetSensor[3] == 1)
    {
        can_transmit_ctrl_data(&canResetSensor[3]);
        sendCanStr.resetSensor[3] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }
}

void send_reset_board_msg(void)
{
    if (sendCanStr.resetBoard[0] == 1)
    {
        can_transmit_ctrl_data(&canResetBoard[0]);
        sendCanStr.resetBoard[0] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetBoard[1] == 1)
    {
        can_transmit_ctrl_data(&canResetBoard[1]);
        sendCanStr.resetBoard[1] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetBoard[2] == 1)
    {
        can_transmit_ctrl_data(&canResetBoard[2]);
        sendCanStr.resetBoard[2] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }

    if (sendCanStr.resetBoard[3] == 1)
    {
        can_transmit_ctrl_data(&canResetBoard[3]);
        sendCanStr.resetBoard[3] = 0;
        vTaskDelay(SEND_CAN_MSG_TMVAL);
    }
}

void can_tx_task_function(void *pvParameters)
{
    //	uint8_t CounterTm = 0;

    while (1)
    {
        send_ws2812b_msg();
        send_reset_sensor_msg();
        send_reset_board_msg();
        vTaskDelay(50);
        taskAliveBits |= TASK_CAN_TX_BIT_4;
    }
}
void can_rx_task_function(void *pvParameters)
{
    uint8_t j;
    while (1)
    {
        if (newMsgFlag == VALID)
        {
            switch (rx_message_struct_g.standard_id)
            {
            case FB_F1_SET_REBOOT_ID:
            case FB_F2_SET_REBOOT_ID:
            case FB_F3_SET_REBOOT_ID:
            case FB_F4_SET_REBOOT_ID:
            {
                parse_reboot_fb_msg();
                break;
            }

            case FB_F1_HEARDBEAT_ID:
            case FB_F2_HEARDBEAT_ID:
            case FB_F3_HEARDBEAT_ID:
            case FB_F4_HEARDBEAT_ID:
            {
                parse_heartbeat_fb_msg();
                break;
            }

            case FB_GET_F1_VERSION_ID:
            case FB_GET_F2_VERSION_ID:
            case FB_GET_F3_VERSION_ID:
            case FB_GET_F4_VERSION_ID:
            {
                parse_version_fb_msg();
                break;
            }

            case FB_SET_F1_WS2812B_ID:
            case FB_SET_F2_WS2812B_ID:
            case FB_SET_F3_WS2812B_ID:
            case FB_SET_F4_WS2812B_ID:
            {
                parse_set_ws2812_fb_msg();
                break;
            }

            case FB_SET_TRAY_F1_WS2812B_ID:
            case FB_SET_TRAY_F2_WS2812B_ID:
            case FB_SET_TRAY_F3_WS2812B_ID:
            case FB_SET_TRAY_F4_WS2812B_ID:
            {
                parse_set_tray_ws2812_fb_msg();
                break;
            }

            case FB_RESET_F1_SENSOR_ID:
            case FB_RESET_F2_SENSOR_ID:
            case FB_RESET_F3_SENSOR_ID:
            case FB_RESET_F4_SENSOR_ID:
            {
                parse_reset_sensor_fb_msg();
                break;
            }

            case FB_GET_F1_UID_ID:
            case FB_GET_F2_UID_ID:
            case FB_GET_F3_UID_ID:
            case FB_GET_F4_UID_ID:
            {
                parse_uid_fb_msg();
                break;
            }

            case RECV_F1_DISTANCE_ID:
            case RECV_F2_DISTANCE_ID:
            case RECV_F3_DISTANCE_ID:
            case RECV_F4_DISTANCE_ID:
            {
                parse_distance_msg();
                break;
            }

            case FB_SYNC_F1_FW_ID:
            case FB_SYNC_F2_FW_ID:
            case FB_SYNC_F3_FW_ID:
            case FB_SYNC_F4_FW_ID:
            {
                parse_sync_fw_fb_msg();
                break;
            }

            default:
                break;
            }
            newMsgFlag = INVALID;
        }

        if (canAliveCounter.loopCounter >= 40)
        {
            canAliveCounter.loopCounter = 0;
            for (j = 0; j < 4; j++)
            {
                if (canAliveCounter.canNowCounter[j] != canAliveCounter.canLastCounter[j])
                {
                    canAliveCounter.canLastCounter[j] = canAliveCounter.canNowCounter[j];
                    canAliveCounter.canTimeoutCounter[j] = 0;
                }
                else
                {
                    canAliveCounter.canTimeoutCounter[j]++;
                }
                if (canAliveCounter.canTimeoutCounter[j] >= 5)
                {
                    distance[j][0] = 0;
                    distance[j][1] = 0;
                    distance[j][2] = 0;
                    canAliveCounter.onLine[j] = 0;
                    canAliveCounter.canTimeoutCounter[j] = 5;
                }
                else
                {
                    canAliveCounter.onLine[j] = 1;
                }
            }
            j = 0;
        }

        canAliveCounter.loopCounter++;
        taskAliveBits |= TASK_CAN_RX_BIT_5;
        vTaskDelay(5);
    }
}

/**
 *  @brief  can1 interrupt function rx0
 *  @param  none
 *  @retval none
 */

void CAN1_RX0_IRQHandler(void)
{
    can_rx_message_type rx_message_struct;

    if (can_flag_get(CAN1, CAN_RF0MN_FLAG) != RESET)
    {
        can_message_receive(CAN1, CAN_RX_FIFO0, &rx_message_struct);

        if ((rx_message_struct.standard_id >= 0x100) && (rx_message_struct.standard_id <= 0x7FF))
        {
            newMsgFlag = VALID;
            memcpy((void *)&rx_message_struct_g, (void *)&rx_message_struct, sizeof(rx_message_struct));
            //  at32_led_toggle( LED3 );
        }
    }
}

/**
 *  @brief  can1 interrupt function se
 *  @param  none
 *  @retval none
 */
void CAN1_SE_IRQHandler(void)
{
    __IO uint32_t err_index = 0;

    if (can_flag_get(CAN1, CAN_ETR_FLAG) != RESET)
    {
        err_index = CAN1->ests & 0x70;
        can_flag_clear(CAN1, CAN_ETR_FLAG);

        /* error type is stuff error */
        if (err_index == 0x00000010)
        {
            /* when stuff error occur: in order to ensure communication normally,
            user must restart can or send a frame of highest priority message here */
        }
    }
}
