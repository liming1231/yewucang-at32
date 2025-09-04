#include "ax_iic.h"

struct SequenceStepEnables
{
    bool tcc, msrc, dss, pre_range, final_range;
};
struct SequenceStepTimeouts
{
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
};

uint32_t measurement_timing_budget_us;

volatile uint8_t stop_variable = 0;
uint8_t hi2c1_tx_buf[I2C1_BUF_SIZE] = {0xC0, 0x88, 0x00, 0x80, 0x01, 0xff, 0x01, 0x00, 0x00, 0x91, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00, 0x60};
uint8_t hi2c1_rx_buf[I2C1_BUF_SIZE] = {0};
i2c_handle_type hi2c1, hi2c2;
i2c_status_type hi2c1_i2c_status;
i2c_status_type hi2c2_i2c_status;

// uint8_t I2C1_ADDRESS = (0x52);
uint8_t I2C1_ADDRESS[3] = {0x52, 0x52, 0x52};
/**
 * @brief  error handler program
 * @param  i2c_status
 * @retval none
 */
void error_handler(uint32_t error_code)
{
    //  while(1)
    //  {
    //    at32_led_toggle(LED2);
    //    delay_ms(500);
    //  }
}

/**
  * @brief  compare whether the valus of buffer 1 and buffer 2 are equal.
  * @param  buffer1: buffer 1 address.
            buffer2: buffer 2 address.
  * @retval 0: equal.
  *         1: unequal.
  */
uint32_t buffer_compare(uint8_t *buffer1, uint8_t *buffer2, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        if (buffer1[i] != buffer2[i])
        {
            return 1;
        }
    }

    return 0;
}

void i2c_lowlevel_init(i2c_handle_type *hi2c)
{
    gpio_init_type gpio_initstructure;

    if (hi2c->i2cx == I2C1_PORT)
    {
        /* i2c periph clock enable */
        crm_periph_clock_enable(I2C1_CLK, TRUE);
        crm_periph_clock_enable(I2C1_SCL_GPIO_CLK, TRUE);
        crm_periph_clock_enable(I2C1_SDA_GPIO_CLK, TRUE);

        /* gpio configuration */
        gpio_initstructure.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        gpio_initstructure.gpio_pull = GPIO_PULL_UP;
        gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
        gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;

        /* configure i2c pins: scl */
        gpio_initstructure.gpio_pins = I2C1_SCL_PIN;
        gpio_init(I2C1_SCL_GPIO_PORT, &gpio_initstructure);

        /* configure i2c pins: sda */
        gpio_initstructure.gpio_pins = I2C1_SDA_PIN;
        gpio_init(I2C1_SDA_GPIO_PORT, &gpio_initstructure);

        /* configure and enable i2c interrupt */
        nvic_irq_enable(I2C1_EVT_IRQn, 0, 0);
        nvic_irq_enable(I2C1_ERR_IRQn, 0, 0);

        i2c_init(hi2c->i2cx, I2C_FSMODE_DUTY_2_1, I2C1_SPEED);

        i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, I2C1_OWN_ADDRESS);
    }
}

void i2c_xshut_init(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the led clock */
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_init_struct.gpio_pins = I2C1_XSHUT_1_PIN;
    gpio_init(GPIOC, &gpio_init_struct);

    gpio_init_struct.gpio_pins = I2C1_XSHUT_2_PIN;
    gpio_init(GPIOC, &gpio_init_struct);

    gpio_init_struct.gpio_pins = I2C1_XSHUT_3_PIN;
    gpio_init(GPIOC, &gpio_init_struct);

    gpio_bits_reset(I2C1_XSHUT_1_PORT, I2C1_XSHUT_1_PIN);
    gpio_bits_reset(I2C1_XSHUT_2_PORT, I2C1_XSHUT_2_PIN);
    gpio_bits_reset(I2C1_XSHUT_3_PORT, I2C1_XSHUT_3_PIN);
}

void write_i2c_bytes(i2c_handle_type *hi2cx, uint8_t index, uint8_t *data, uint16_t size)
{
    if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], data, size, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}
void write_i2c_u8(i2c_handle_type *hi2cx, uint8_t index, uint8_t reg, uint8_t data)
{
    uint8_t toU8[2];
    toU8[0] = reg;
    toU8[1] = data;

    if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], toU8, 2, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}
void write_i2c_u16(i2c_handle_type *hi2cx, uint8_t index, uint8_t reg, uint16_t data)
{
    uint8_t toU8[3];
    toU8[0] = reg;
    toU8[1] = (data >> 8) & 0xFF;
    toU8[2] = (data >> 0) & 0xFF;

    if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], toU8, 3, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}
void write_i2c_u32(i2c_handle_type *hi2cx, uint8_t index, uint8_t reg, uint32_t data)
{
    uint8_t toU8[5];
    toU8[0] = reg;
    toU8[1] = (data >> 24) & 0xFF;
    toU8[2] = (data >> 16) & 0xFF;
    toU8[3] = (data >> 8) & 0xFF;
    toU8[4] = (data >> 0) & 0xFF;

    if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], toU8, 5, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}

void write_i2c_reg(i2c_handle_type *hi2cx, uint8_t index, uint8_t reg)
{
    uint8_t toU8[2];
    toU8[0] = reg;

    if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], toU8, 1, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}

uint8_t toSend[16] = {0};
void write_i2c_reg_data(i2c_handle_type *hi2cx, uint8_t index, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t i;
    toSend[0] = reg;

    if (len <= 16)
    {
        for (i = 0; i < len; i++)
        {
            toSend[i + 1] = data[i];
        }

        if ((hi2c1_i2c_status = i2c_master_transmit_int(hi2cx, I2C1_ADDRESS[index - 1], toSend, len + 1, I2C_TIMEOUT)) != I2C_OK)
        {
            error_handler(hi2c1_i2c_status);
        }

        if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
        {
            error_handler(hi2c1_i2c_status);
        }

        vTaskDelay(10);
    }
}

void read_i2c_bytes(i2c_handle_type *hi2cx, uint8_t index, uint8_t *data, uint16_t size)
{
    if ((hi2c1_i2c_status = i2c_master_receive_int(hi2cx, I2C1_ADDRESS[index - 1], data, size, I2C_TIMEOUT)) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    if (i2c_wait_end(hi2cx, I2C_TIMEOUT) != I2C_OK)
    {
        error_handler(hi2c1_i2c_status);
    }

    vTaskDelay(10);
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool get_spad_info(uint8_t *count, uint8_t index, bool *type_is_aperture)
{
    write_i2c_u8(&hi2c1, index, 0x80, 0x01);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
    write_i2c_u8(&hi2c1, index, 0x00, 0x00);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x06);

    write_i2c_reg(&hi2c1, index, 0x83);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
    write_i2c_u8(&hi2c1, index, 0x83, (hi2c1_rx_buf[0] | 0x04));

    write_i2c_u8(&hi2c1, index, 0xFF, 0x07);
    write_i2c_u8(&hi2c1, index, 0x81, 0x01);
    write_i2c_u8(&hi2c1, index, 0x80, 0x01);
    write_i2c_u8(&hi2c1, index, 0x94, 0x6B);
    write_i2c_u8(&hi2c1, index, 0x83, 0x00);

    write_i2c_reg(&hi2c1, index, 0x83);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);

    if (hi2c1_rx_buf[0] == 0)
    {
        return false;
    }

    write_i2c_u8(&hi2c1, index, 0x83, 0x01);

    write_i2c_reg(&hi2c1, index, 0x92);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
    *count = hi2c1_rx_buf[0] & 0x7f;
    *type_is_aperture = (hi2c1_rx_buf[0] >> 7) & 0x01;
    write_i2c_u8(&hi2c1, index, 0x81, 0x00);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x06);

    write_i2c_reg(&hi2c1, index, 0x83);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
    write_i2c_u8(&hi2c1, index, 0x83, (hi2c1_rx_buf[0] & ~0x04));

    write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
    write_i2c_u8(&hi2c1, index, 0x00, 0x01);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
    write_i2c_u8(&hi2c1, index, 0x80, 0x00);

    return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(uint8_t index, enum vcselPeriodType type)
{
    if (type == VcselPeriodPreRange)
    {
        write_i2c_reg(&hi2c1, index, 0x50);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
        return (((hi2c1_rx_buf[0]) + 1) << 1);
    }

    else if (type == VcselPeriodFinalRange)
    {
        write_i2c_reg(&hi2c1, index, 0x70);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
        return (((hi2c1_rx_buf[0]) + 1) << 1);
    }

    else
    {
        return 255;
    }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

void getSequenceStepEnables(uint8_t index, struct SequenceStepEnables *enables)
{
    uint8_t sequence_config;
    write_i2c_reg(&hi2c1, index, 0x01);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
    sequence_config = hi2c1_rx_buf[0];

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(uint8_t index, struct SequenceStepEnables const *enables, struct SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(index, VcselPeriodPreRange);

    write_i2c_reg(&hi2c1, index, 0x46);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
    timeouts->msrc_dss_tcc_mclks = hi2c1_rx_buf[0] + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                                           timeouts->pre_range_vcsel_period_pclks);

    write_i2c_reg(&hi2c1, index, 0x51);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 2);
    timeouts->pre_range_mclks = decodeTimeout(((hi2c1_rx_buf[0] << 8) | hi2c1_rx_buf[1]) & 0xFFFF);
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                                        timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(index, VcselPeriodFinalRange);

    write_i2c_reg(&hi2c1, index, 0x71);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 2);
    timeouts->final_range_mclks = decodeTimeout(((hi2c1_rx_buf[0] << 8) | hi2c1_rx_buf[1]) & 0xFFFF);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                   timeouts->final_range_vcsel_period_pclks);
}

uint32_t getMeasurementTimingBudget(uint8_t index)
{
    struct SequenceStepEnables enables;
    struct SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(index, &enables);
    getSequenceStepTimeouts(index, &enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }

    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t encodeTimeout(uint32_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }

    else
    {
        return 0;
    }
}

bool setMeasurementTimingBudget(uint8_t index, uint32_t budget_us)
{
    struct SequenceStepEnables enables;
    struct SequenceStepTimeouts timeouts;

    uint32_t final_range_timeout_us;
    uint32_t final_range_timeout_mclks;

    uint16_t const StartOverhead = 1910;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(index, &enables);
    getSequenceStepTimeouts(index, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }

    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us)
        {
            // "Requested timeout too big."
            return false;
        }

        final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us,
                                                               timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        write_i2c_u16(&hi2c1, index, 0x71, encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurement_timing_budget_us = budget_us; // store for internal reuse
    }

    return true;
}

// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t index, uint8_t vhv_init_byte)
{
    write_i2c_u8(&hi2c1, index, 0x00, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    // startTimeout();
    write_i2c_reg(&hi2c1, index, 0x13);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);

    if ((hi2c1_rx_buf[0] & 0x07) == 0)
    {
        return false;
    }

    write_i2c_u8(&hi2c1, index, 0x0B, 0x01);
    write_i2c_u8(&hi2c1, index, 0x00, 0x00);

    return true;
}

bool initVl53l0x(uint8_t index)
{
    uint8_t i;
    uint8_t spad_count;
    bool spad_type_is_aperture;
    uint8_t first_spad_to_enable;
    uint8_t spads_enabled = 0;
    uint8_t ref_spad_map[6];

    if (index == 1)
    {
        gpio_bits_set(I2C1_XSHUT_1_PORT, I2C1_XSHUT_1_PIN);
    }

    else if (index == 2)
    {
        gpio_bits_set(I2C1_XSHUT_2_PORT, I2C1_XSHUT_2_PIN);
    }

    else if (index == 3)
    {
        gpio_bits_set(I2C1_XSHUT_3_PORT, I2C1_XSHUT_3_PIN);
    }

    else
    {
        return false;
    }

    vTaskDelay(50);

    /* ��ȡ�豸��ʶ */
    write_i2c_reg(&hi2c1, index, 0xC0);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);

    if (hi2c1_rx_buf[0] != 0xEE)
    {
        return false;
    }

    else
    {
        /* ����i2c��׼ģʽ */
        write_i2c_u8(&hi2c1, index, 0x88, 0x00);
        /* �����豸i2c��ַ */
        write_i2c_u8(&hi2c1, index, 0x8A, (0x52 + index * 2) / 2);
        I2C1_ADDRESS[index - 1] = (0x52 + index * 2);
        write_i2c_u8(&hi2c1, index, 0x80, 0x01);
        write_i2c_u8(&hi2c1, index, 0xff, 0x01);
        write_i2c_u8(&hi2c1, index, 0x00, 0x00);
        write_i2c_reg(&hi2c1, index, 0x91);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
        stop_variable = hi2c1_rx_buf[0];
        write_i2c_u8(&hi2c1, index, 0x00, 0x01);
        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x80, 0x00);
        write_i2c_reg(&hi2c1, index, 0x60);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        write_i2c_reg(&hi2c1, index, hi2c1_rx_buf[0] | 0x12);
        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        write_i2c_u16(&hi2c1, index, 0x44, (uint16_t)(0.25 * (1 << 7)));
        write_i2c_u8(&hi2c1, index, 0x01, 0xFF);

        if (!get_spad_info(&spad_count, index, &spad_type_is_aperture))
        {
            return false;
        }

        write_i2c_reg(&hi2c1, index, 0xB0);
        read_i2c_bytes(&hi2c1, index, ref_spad_map, 6);
        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x4F, 0x00);
        write_i2c_u8(&hi2c1, index, 0x4E, 0x2C);
        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0xB6, 0xB4);

        first_spad_to_enable = spad_type_is_aperture ? 12 : 0;

        for (i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count)
            {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }

            else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
                spads_enabled++;
            }
        }

        write_i2c_reg_data(&hi2c1, index, 0xB0, ref_spad_map, 6);
        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x00, 0x00);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x09, 0x00);
        write_i2c_u8(&hi2c1, index, 0x10, 0x00);
        write_i2c_u8(&hi2c1, index, 0x11, 0x00);

        write_i2c_u8(&hi2c1, index, 0x24, 0x01);
        write_i2c_u8(&hi2c1, index, 0x25, 0xFF);
        write_i2c_u8(&hi2c1, index, 0x75, 0x00);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x4E, 0x2C);
        write_i2c_u8(&hi2c1, index, 0x48, 0x00);
        write_i2c_u8(&hi2c1, index, 0x30, 0x20);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x30, 0x09);
        write_i2c_u8(&hi2c1, index, 0x54, 0x00);
        write_i2c_u8(&hi2c1, index, 0x31, 0x04);
        write_i2c_u8(&hi2c1, index, 0x32, 0x03);
        write_i2c_u8(&hi2c1, index, 0x40, 0x83);
        write_i2c_u8(&hi2c1, index, 0x46, 0x25);
        write_i2c_u8(&hi2c1, index, 0x60, 0x00);
        write_i2c_u8(&hi2c1, index, 0x27, 0x00);
        write_i2c_u8(&hi2c1, index, 0x50, 0x06);
        write_i2c_u8(&hi2c1, index, 0x51, 0x00);
        write_i2c_u8(&hi2c1, index, 0x52, 0x96);
        write_i2c_u8(&hi2c1, index, 0x56, 0x08);
        write_i2c_u8(&hi2c1, index, 0x57, 0x30);
        write_i2c_u8(&hi2c1, index, 0x61, 0x00);
        write_i2c_u8(&hi2c1, index, 0x62, 0x00);
        write_i2c_u8(&hi2c1, index, 0x64, 0x00);
        write_i2c_u8(&hi2c1, index, 0x65, 0x00);
        write_i2c_u8(&hi2c1, index, 0x66, 0xA0);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x22, 0x32);
        write_i2c_u8(&hi2c1, index, 0x47, 0x14);
        write_i2c_u8(&hi2c1, index, 0x49, 0xFF);
        write_i2c_u8(&hi2c1, index, 0x4A, 0x00);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x7A, 0x0A);
        write_i2c_u8(&hi2c1, index, 0x7B, 0x00);
        write_i2c_u8(&hi2c1, index, 0x78, 0x21);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x23, 0x34);
        write_i2c_u8(&hi2c1, index, 0x42, 0x00);
        write_i2c_u8(&hi2c1, index, 0x44, 0xFF);
        write_i2c_u8(&hi2c1, index, 0x45, 0x26);
        write_i2c_u8(&hi2c1, index, 0x46, 0x05);
        write_i2c_u8(&hi2c1, index, 0x40, 0x40);
        write_i2c_u8(&hi2c1, index, 0x0E, 0x06);
        write_i2c_u8(&hi2c1, index, 0x20, 0x1A);
        write_i2c_u8(&hi2c1, index, 0x43, 0x40);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x34, 0x03);
        write_i2c_u8(&hi2c1, index, 0x35, 0x44);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x31, 0x04);
        write_i2c_u8(&hi2c1, index, 0x4B, 0x09);
        write_i2c_u8(&hi2c1, index, 0x4C, 0x05);
        write_i2c_u8(&hi2c1, index, 0x4D, 0x04);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x44, 0x00);
        write_i2c_u8(&hi2c1, index, 0x45, 0x20);
        write_i2c_u8(&hi2c1, index, 0x47, 0x08);
        write_i2c_u8(&hi2c1, index, 0x48, 0x28);
        write_i2c_u8(&hi2c1, index, 0x67, 0x00);
        write_i2c_u8(&hi2c1, index, 0x70, 0x04);
        write_i2c_u8(&hi2c1, index, 0x71, 0x01);
        write_i2c_u8(&hi2c1, index, 0x72, 0xFE);
        write_i2c_u8(&hi2c1, index, 0x76, 0x00);
        write_i2c_u8(&hi2c1, index, 0x77, 0x00);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x0D, 0x01);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x80, 0x01);
        write_i2c_u8(&hi2c1, index, 0x01, 0xF8);

        write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
        write_i2c_u8(&hi2c1, index, 0x8E, 0x01);
        write_i2c_u8(&hi2c1, index, 0x00, 0x01);
        write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
        write_i2c_u8(&hi2c1, index, 0x80, 0x00);

        write_i2c_u8(&hi2c1, index, 0x0A, 0x04);

        write_i2c_reg(&hi2c1, index, 0x84);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 1);
        write_i2c_u8(&hi2c1, index, 0x84, (hi2c1_rx_buf[0] & ~0x10));

        write_i2c_u8(&hi2c1, index, 0x0B, 0x01);

        measurement_timing_budget_us = getMeasurementTimingBudget(index);
        write_i2c_u8(&hi2c1, index, 0x01, 0xE8);

        setMeasurementTimingBudget(index, measurement_timing_budget_us);

        write_i2c_u8(&hi2c1, index, 0x01, 0x01);

        if (!performSingleRefCalibration(index, 0x40))
        {
            return false;
        }

        write_i2c_u8(&hi2c1, index, 0x01, 0x02);

        if (!performSingleRefCalibration(index, 0x00))
        {
            return false;
        }

        write_i2c_u8(&hi2c1, index, 0x01, 0xE8);

        return true;
    }
}

void startContinuous(uint8_t index, uint32_t period_ms)
{
    uint16_t osc_calibrate_val;
    write_i2c_u8(&hi2c1, index, 0x80, 0x01);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x01);
    write_i2c_u8(&hi2c1, index, 0x00, 0x00);
    write_i2c_u8(&hi2c1, index, 0x91, stop_variable);
    write_i2c_u8(&hi2c1, index, 0x00, 0x01);
    write_i2c_u8(&hi2c1, index, 0xFF, 0x00);
    write_i2c_u8(&hi2c1, index, 0x80, 0x00);

    if (period_ms != 0)
    {
        // continuous timed mode
        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
        write_i2c_reg(&hi2c1, index, 0xF8);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 2);
        osc_calibrate_val = ((hi2c1_rx_buf[0] << 8) | hi2c1_rx_buf[1]) & 0xFFFF;

        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }

        write_i2c_u32(&hi2c1, index, 0x04, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
        write_i2c_u8(&hi2c1, index, 0x00, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }

    else
    {
        // continuous back-to-back mode
        write_i2c_u8(&hi2c1, index, 0x00, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

uint16_t readRangeContinuousMillimeters(uint8_t index)
{
    uint16_t value;
    // startTimeout();
    write_i2c_reg(&hi2c1, index, 0x13);
    read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 2);

    if ((hi2c1_rx_buf[0] & 0x07) == 0)
    {
        return 0xffff;
    }

    else
    {
        write_i2c_reg(&hi2c1, index, 0x1E);
        read_i2c_bytes(&hi2c1, index, hi2c1_rx_buf, 2);
        value = ((hi2c1_rx_buf[0] << 8) | hi2c1_rx_buf[1]) & 0xFFFF;
        write_i2c_u8(&hi2c1, index, 0x0B, 0x01);
        return value;
    }
}
