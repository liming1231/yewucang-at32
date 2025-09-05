#include "ax_common.h"

EventBits_t taskAliveBits = 0;

volatile uint16_t distance[4][6] = {0};
uint8_t versionSub[4][6] = {0};
uint8_t uidBuf[5][12] = {0};
struct uart_send_flag uart1SendTypeFlag = {0};
struct _send_can_str sendCanStr = {0};

can_tx_message_type canResetSensor[4] = {0};
can_tx_message_type canSetLeds[4] = {0};
can_tx_message_type canResetBoard[4] = {0};

uint16_t crc16_modbus(uint8_t *pszBuf, uint8_t unLength)
{
    int i;
    uint16_t crc_t = 0xFFFF;
    uint8_t CRC_count;
    for (CRC_count = 0; CRC_count < unLength; CRC_count++)
    {
        crc_t = crc_t ^ *(pszBuf + CRC_count);
        for (i = 0; i < 8; i++)
        {
            if (crc_t & 1)
            {
                crc_t >>= 1;
                crc_t ^= 0xA001;
            }
            else
            {
                crc_t >>= 1;
            }
        }
    }
    return crc_t;
}
