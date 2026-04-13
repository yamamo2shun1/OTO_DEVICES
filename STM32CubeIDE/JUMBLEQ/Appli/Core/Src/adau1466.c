/*
 * adau1466.c
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#include "adau1466.h"

#include "SigmaStudioFW.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1_Defines.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1_PARAM.h"

#include "cmsis_os2.h"
#include <string.h>

#define ADAU1466_REG_PLL_ENABLE 0xF003U
#define ADAU1466_REG_PLL_LOCK   0xF004U
#define ADAU1466_REG_MCLK_OUT   0xF005U
#define ADAU1466_REG_CLK_GEN1_M 0xF020U

#define ADAU1466_PLL_LOCK_TIMEOUT_MS    200U
#define ADAU1466_DEFAULT_SAMPLE_RATE_HZ 48000U

typedef struct
{
    uint8_t clk_gen1_m;
    uint8_t mclk_out;
} adau1466_sample_rate_cfg_t;

static uint32_t adau1466_sample_rate_hz = ADAU1466_DEFAULT_SAMPLE_RATE_HZ;

static bool adau1466_get_sample_rate_cfg(uint32_t hz, adau1466_sample_rate_cfg_t* cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    if (hz == 48000U)
    {
        cfg->clk_gen1_m = 0x06U;
        cfg->mclk_out   = 0x05U;
        return true;
    }

    if (hz == 96000U)
    {
        cfg->clk_gen1_m = 0x03U;
        cfg->mclk_out   = 0x07U;
        return true;
    }

    return false;
}

static void adau1466_write_reg_u16(uint16_t addr, uint8_t value)
{
    uint8_t data[2] = {0x00, value};
    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 2, data);
}

static bool adau1466_wait_pll_lock(uint32_t timeout_ms)
{
    uint8_t pll_lock[2] = {0};
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        SIGMA_READ_REGISTER(DEVICE_ADDR_ADAU146XSCHEMATIC_1, ADAU1466_REG_PLL_LOCK, 2, pll_lock);
        if ((pll_lock[1] & 0x01U) != 0U)
        {
            return true;
        }
        osDelay(1);
    }

    SEGGER_RTT_printf(0, "[ADAU1466] PLL lock timeout\n");
    return false;
}

static uint32_t adau1466_q8_24_to_raw(double val)
{
    int64_t fixed_q8_24 = (int64_t) llround(val * 16777216.0);  // 2^24
    if (fixed_q8_24 > INT32_MAX)
    {
        fixed_q8_24 = INT32_MAX;
    }
    else if (fixed_q8_24 < INT32_MIN)
    {
        fixed_q8_24 = INT32_MIN;
    }

    return (uint32_t) ((int32_t) fixed_q8_24);
}

static void adau1466_store_be32(uint32_t raw, uint8_t out[4])
{
    out[0] = (uint8_t) ((raw >> 24) & 0xFFU);
    out[1] = (uint8_t) ((raw >> 16) & 0xFFU);
    out[2] = (uint8_t) ((raw >> 8) & 0xFFU);
    out[3] = (uint8_t) (raw & 0xFFU);
}

static void adau1466_delay_us(uint32_t delay_us)
{
    if (delay_us == 0U)
    {
        return;
    }

    if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
    {
        HAL_Delay((delay_us + 999U) / 1000U);
        return;
    }

    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (cycles_per_us == 0U)
    {
        HAL_Delay(1);
        return;
    }

    uint32_t start_cycle = DWT->CYCCNT;
    uint32_t wait_cycles = cycles_per_us * delay_us;

    while ((DWT->CYCCNT - start_cycle) < wait_cycles)
    {
    }
}

static void adau1466_wait_safeload_frame(void)
{
    uint32_t sample_rate_hz = adau1466_sample_rate_hz;
    if (sample_rate_hz == 0U)
    {
        sample_rate_hz = ADAU1466_DEFAULT_SAMPLE_RATE_HZ;
    }

    adau1466_delay_us((1000000U + sample_rate_hz - 1U) / sample_rate_hz);
}

static bool adau1466_safeload_write_words(uint16_t addr, uint8_t mem_page, const uint8_t* data, uint8_t word_count)
{
    uint8_t safeload_ctrl[12] = {0x00};

    if ((data == NULL) || (word_count == 0U) || (word_count > 5U))
    {
        SEGGER_RTT_printf(0, "[ADAU1466] invalid safeload word_count: %u\n", word_count);
        return false;
    }

    if (mem_page > 1U)
    {
        SEGGER_RTT_printf(0, "[ADAU1466] invalid safeload mem_page: %u\n", mem_page);
        return false;
    }

    adau1466_store_be32(addr, &safeload_ctrl[0]);
    if (mem_page == 0U)
    {
        safeload_ctrl[7] = word_count;
    }
    else
    {
        safeload_ctrl[11] = word_count;
    }

    SIGMA_SAFELOAD_WRITE_DATA(
        DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SAFELOAD_DATA_SAFELOAD0_ADDR, (uint16_t) (word_count * 4U), (uint8_t*) data);
    SIGMA_SAFELOAD_WRITE_DATA(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SAFELOAD_ADDR_SAFELOAD_ADDR, sizeof(safeload_ctrl), safeload_ctrl);
    adau1466_wait_safeload_frame();

    return true;
}

double convert_pot2dB(uint16_t adc_val)
{
    double x  = (double) adc_val / 1023.0;
    double db = 0.0;
    if (x < 0.7)
    {
        db = -80.0 + (x / 0.7) * 80.0;
    }
    else
    {
        db = (x - 0.7) / 0.3 * 10.0;
    }
    return db;
}

int16_t convert_pot2dB_int(uint16_t adc_val)
{
    // Pot end-stop付近のADCノイズで表示/制御値が揺れないように端点デッドゾーンを設ける
    if (adc_val <= 5U)
    {
        return -80;
    }
    if (adc_val >= (1023U - 5U))
    {
        return 10;
    }

    double db    = convert_pot2dB(adc_val);
    int16_t db_i = (int16_t) ((db >= 0.0) ? (db + 0.5) : (db - 0.5));
    if (db_i < -80)
    {
        db_i = -80;
    }
    if (db_i > 10)
    {
        db_i = 10;
    }
    return db_i;
}

double convert_dB2gain(double db)
{
    return pow(10.0, db / 20.0);
}

static void write_q8_24(const uint16_t addr, const double val)
{
    uint8_t gain_array[4] = {0x00};
    uint32_t raw          = adau1466_q8_24_to_raw(val);

    adau1466_store_be32(raw, gain_array);

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 4, gain_array);
}

void safeload_write_q8_24(uint16_t addr, uint8_t mem_page, double val)
{
    uint8_t safeload_data[4] = {0x00};
    adau1466_store_be32(adau1466_q8_24_to_raw(val), safeload_data);
    (void) adau1466_safeload_write_words(addr, mem_page, safeload_data, 1U);
}

void AUDIO_Init_ADAU1466(uint32_t hz)
{
    // ADAU1466 HW Reset
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 0);
    osDelay(10);
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 1);
    osDelay(500);

    (void) AUDIO_Update_ADAU1466_SampleRate(hz);
}

bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz)
{
    adau1466_sample_rate_cfg_t cfg;

    if (!adau1466_get_sample_rate_cfg(hz, &cfg))
    {
        SEGGER_RTT_printf(0, "[ADAU1466] unsupported sample rate: %lu\n", (unsigned long) hz);
        return false;
    }

    // Re-run SigmaStudio default register/program sequence without HW reset.
    // This keeps runtime update deterministic and aligns with known-good init flow.
#if RESET_FROM_FW
    default_download_ADAU146XSCHEMATIC_1();
    osDelay(5);
#endif

    // Update PLL-related clock generation for selected sample rate.
    adau1466_write_reg_u16(ADAU1466_REG_CLK_GEN1_M, cfg.clk_gen1_m);
    adau1466_write_reg_u16(ADAU1466_REG_MCLK_OUT, cfg.mclk_out);

    // Re-enable PLL and wait for lock.
    adau1466_write_reg_u16(ADAU1466_REG_PLL_ENABLE, 0x00U);
    __DSB();
    osDelay(1);
    adau1466_write_reg_u16(ADAU1466_REG_PLL_ENABLE, 0x01U);

    bool pll_locked = adau1466_wait_pll_lock(ADAU1466_PLL_LOCK_TIMEOUT_MS);
    if (pll_locked)
    {
        adau1466_sample_rate_hz = hz;
    }

    return pll_locked;
}

void set_dc_inputA(float xf_pos)
{
    write_q8_24(MOD_DCINPUT_A_DCVALUE_ADDR, xf_pos);
}

void set_dc_inputB(float xf_pos)
{
    write_q8_24(MOD_DCINPUT_B_DCVALUE_ADDR, xf_pos);
}

void control_input_from_usb_gain(uint8_t ch, int16_t db)
{
    SEGGER_RTT_printf(0, "USB CH%d Gain: %.2f dB\n", ch, db);

    const double gain = convert_dB2gain(db);

    switch (ch)
    {
    case 1:
        write_q8_24(MOD_INPUT_FROM_USB1_GAIN_ADDR, gain);
        break;
    case 2:
        write_q8_24(MOD_INPUT_FROM_USB2_GAIN_ADDR, gain);
        break;
    case 3:
        write_q8_24(MOD_INPUT_FROM_USB3_GAIN_ADDR, gain);
        break;
    case 4:
        write_q8_24(MOD_INPUT_FROM_USB4_GAIN_ADDR, gain);
        break;
    default:
        break;
    }
}

void control_input_from_ch1_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH1_GAIN_ADDR, gain);
}

void control_input_from_ch2_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH2_GAIN_ADDR, gain);
}

void control_input_from_return_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_RETURN_GAIN_ADDR, gain);
}

void control_send1_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND1_OUTPUT_GAIN_ADDR, gain);
}

void control_send2_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND2_OUTPUT_GAIN_ADDR, gain);
}

void control_dryA_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYA_DCVALUE_ADDR, rate);
}

void control_dryB_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYB_DCVALUE_ADDR, rate);
}

void control_wet_out_gain(const uint16_t adc_val)
{
    const float rate = sin(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_WET_DCVALUE_ADDR, rate);
}

void control_ch1_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_CH1_OUTPUT_GAIN_ADDR, gain);
}

void control_ch2_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_CH2_OUTPUT_GAIN_ADDR, gain);
}

void control_hp_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_HP_OUTPUT_GAIN_ADDR, gain);
}

void set_ch1_line()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch1_phono()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch2_line()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch2_phono()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void select_input_type(uint8_t ch, uint8_t type)
{
    if (ch == INPUT_CH1)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch1_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch1_phono();
            break;
        default:
            break;
        }
    }
    else if (ch == INPUT_CH2)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch2_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch2_phono();
            break;
        default:
            break;
        }
    }
}

void disable_ch1_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_1_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void enable_ch1_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_1_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void select_send_ch1_src(bool enable)
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, enable ? 0x01 : 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SEND_SW_1_INDEX_ADDR, 4, Mode0_0);
}

void disable_ch2_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_2_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void enable_ch2_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_2_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void select_send_ch2_src(bool enable)
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, enable ? 0x01 : 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SEND_SW_2_INDEX_ADDR, 4, Mode0_0);
}

void enable_dvs(uint8_t ch, bool enable)
{
    if (ch == INPUT_CH1)
    {
        if (enable)
        {
            enable_ch1_dvs();
        }
        else
        {
            disable_ch1_dvs();
        }
        select_send_ch1_src(enable);
    }
    else if (ch == INPUT_CH2)
    {
        if (enable)
        {
            enable_ch2_dvs();
        }
        else
        {
            disable_ch2_dvs();
        }
        select_send_ch2_src(enable);
    }
}

void select_xf_assignA_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_A_INDEX_ADDR, 4, Mode0);
}

void select_xf_assignB_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_B_INDEX_ADDR, 4, Mode0);
}

void select_xf_assignPost_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_POST_INDEX_ADDR, 4, Mode0);
}

void select_return_ch_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_USB12:
        Mode0[3] = 0x00;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x01;
        break;
    default:
        return;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_RETURN_CH_SW_INDEX_ADDR, 4, Mode0);
}

void select_hp_out_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case CUE_SEL_XF_A:
        Mode0[3] = 0x00;
        break;
    case CUE_SEL_XF_B:
        Mode0[3] = 0x01;
        break;
    case CUE_SEL_THRU:
        Mode0[3] = 0x02;
        break;
    case CUE_SEL_MST:
        Mode0[3] = 0x03;
        break;
    default:
    	break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_HP_OUT_SW_INDEX_ADDR, 4, Mode0);
}
