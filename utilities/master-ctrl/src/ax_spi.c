#include "ax_spi.h"
#include <string.h>

// uint8_t spi2_tx_buffer2[8*3*60]={0};

uint8_t spi2_tx_buffer[8 * 3 * 3] =
    {
        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
        0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,

        0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,

        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
        0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
        0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC};

spi_init_type spi_init_struct;

void spi2_gpio_cfg(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    /* can tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = SPI2_MOSI_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(SPI2_MOSI_PORT, &gpio_init_struct);
}

void spi2_config(void)
{
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI2, &spi_init_struct);

    spi_enable(SPI2, TRUE);
}

void spi2_send_byte(uint8_t d)
{
    while (spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET)
        ;

    spi_i2s_data_transmit(SPI2, d);
}

void spi2_send_bytes(uint8_t *datas, uint16_t len)
{
    uint16_t spiDataIndex = 0;

    while (spiDataIndex < len)
    {
        spi2_send_byte(datas[spiDataIndex]);
        spiDataIndex++;
    }
}

#if 0
void spi_tx_task_function( void *pvParameters )
{
    int i = 0;

    while( 1 )
    {
        for( i = 0; i<20; i++ )
        {
            memcpy( ( void * )&spi2_tx_buffer2[i*8*3*3],spi2_tx_buffer,8*3*3 );
        }

        spi2_send_bytes( spi2_tx_buffer2, 8*3*60 );
        vTaskDelay( 100 );
    }
}
#endif
