#include "cc1101.h"
#include "smartrf_cc1101.h"

#include "main.h"
#include <stddef.h>

#define CC1101_SPI_TIMEOUT_MS     20U
#define CC1101_MISO_WAIT_TIMEOUT  10U

#define PRINCETON_BITS                 24U
#define PRINCETON_SHORT_MIN_US         180U
#define PRINCETON_SHORT_MAX_US         700U
#define PRINCETON_LONG_MIN_US          700U
#define PRINCETON_LONG_MAX_US          1900U
#define PRINCETON_SYNC_GAP_MIN_US      4500U

#define FLIPPER_CODE_FORWARD_24   0xC30111UL
#define FLIPPER_CODE_BACKWARD_24  0xC30222UL
#define FLIPPER_CODE_LEFT_24      0xC30333UL
#define FLIPPER_CODE_RIGHT_24     0xC30444UL
#define FLIPPER_CODE_CENTER_24    0xC30555UL
#define FLIPPER_CODE_STOP_24      0xC30FFFUL

typedef struct {
    uint8_t reg;
    uint8_t val;
} CC1101_RegPair_t;

typedef struct {
    uint32_t code24;
    CC1101_Command_t cmd;
} CC1101_CodeCommandMap_t;

static SPI_HandleTypeDef* s_hspi = NULL;

typedef struct {
    uint8_t has_prev_edge;
    uint8_t prev_level;
    uint8_t bit_count;
    uint32_t prev_ts_us;
    uint32_t high_us;
    uint32_t low_us;
    uint32_t assembled_code;
} PrincetonDecoderState_t;

static PrincetonDecoderState_t s_princeton = {0};

static const CC1101_CodeCommandMap_t k_code_command_map[] = {
    {FLIPPER_CODE_FORWARD_24, CC1101_CMD_FORWARD},
    {FLIPPER_CODE_BACKWARD_24, CC1101_CMD_BACKWARD},
    {FLIPPER_CODE_LEFT_24, CC1101_CMD_LEFT},
    {FLIPPER_CODE_RIGHT_24, CC1101_CMD_RIGHT},
    {FLIPPER_CODE_CENTER_24, CC1101_CMD_CENTER},
    {FLIPPER_CODE_STOP_24, CC1101_CMD_STOP}
};

static HAL_StatusTypeDef CC1101_WaitMisoLow(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();
    while (HAL_GPIO_ReadPin(CC1101_MISO_GPIO_Port, CC1101_MISO_Pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start_ms) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static inline void CC1101_Select(void)
{
    HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_RESET);
}

static inline void CC1101_Deselect(void)
{
    HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef CC1101_SpiTxRxByte(uint8_t tx, uint8_t* rx)
{
    uint8_t tx_buf = tx;
    uint8_t rx_buf = 0U;
    HAL_StatusTypeDef hal = HAL_SPI_TransmitReceive(s_hspi, &tx_buf, &rx_buf, 1U, CC1101_SPI_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return hal;
    }
    if (rx != NULL) {
        *rx = rx_buf;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t* status)
{
    uint8_t st = 0U;
    HAL_StatusTypeDef hal;

    CC1101_Select();
    hal = CC1101_WaitMisoLow(CC1101_MISO_WAIT_TIMEOUT);
    if (hal == HAL_OK) {
        hal = CC1101_SpiTxRxByte(strobe, &st);
    }
    CC1101_Deselect();

    if ((hal == HAL_OK) && (status != NULL)) {
        *status = st;
    }
    return hal;
}

static HAL_StatusTypeDef CC1101_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal;

    CC1101_Select();
    hal = CC1101_WaitMisoLow(CC1101_MISO_WAIT_TIMEOUT);
    if (hal != HAL_OK) {
        CC1101_Deselect();
        return hal;
    }
    hal = CC1101_SpiTxRxByte(reg, NULL);
    if (hal == HAL_OK) {
        hal = CC1101_SpiTxRxByte(value, NULL);
    }
    CC1101_Deselect();
    return hal;
}

static uint8_t CC1101_IsShortPulse(uint32_t duration_us)
{
    return ((duration_us >= PRINCETON_SHORT_MIN_US) && (duration_us <= PRINCETON_SHORT_MAX_US)) ? 1U : 0U;
}

static uint8_t CC1101_IsLongPulse(uint32_t duration_us)
{
    return ((duration_us >= PRINCETON_LONG_MIN_US) && (duration_us <= PRINCETON_LONG_MAX_US)) ? 1U : 0U;
}

static void Princeton_Reset(PrincetonDecoderState_t* st)
{
    st->bit_count = 0U;
    st->assembled_code = 0U;
    st->high_us = 0U;
    st->low_us = 0U;
}

static uint32_t Princeton_Reverse24(uint32_t value)
{
    uint32_t rev = 0U;
    uint8_t i;
    for (i = 0U; i < PRINCETON_BITS; i++) {
        rev <<= 1U;
        rev |= (value & 0x1U);
        value >>= 1U;
    }
    return rev & 0xFFFFFFUL;
}

static uint8_t Princeton_DecodeSymbol(PrincetonDecoderState_t* st, uint8_t* out_bit)
{
    if (CC1101_IsShortPulse(st->high_us) && CC1101_IsLongPulse(st->low_us)) {
        *out_bit = 0U;
        return 1U;
    }
    if (CC1101_IsLongPulse(st->high_us) && CC1101_IsShortPulse(st->low_us)) {
        *out_bit = 1U;
        return 1U;
    }
    return 0U;
}

static CC1101_Command_t CC1101_MapCodeToCommand(uint32_t code24)
{
    uint32_t i;
    uint32_t code = code24 & 0xFFFFFFUL;
    uint32_t count = (uint32_t)(sizeof(k_code_command_map) / sizeof(k_code_command_map[0]));

    for (i = 0U; i < count; i++) {
        if (k_code_command_map[i].code24 == code) {
            return k_code_command_map[i].cmd;
        }
    }
    return CC1101_CMD_NONE;
}

static void CC1101_ApplyResetPulseSequence(void)
{
    CC1101_Deselect();
    HAL_Delay(1U);
    CC1101_Select();
    HAL_Delay(1U);
    CC1101_Deselect();
    HAL_Delay(1U);
}

static HAL_StatusTypeDef CC1101_EnterIdleAndFlushRx(void)
{
    HAL_StatusTypeDef hal;

    hal = CC1101_Strobe(CC1101_STROBE_SIDLE, NULL);
    if (hal != HAL_OK) {
        return hal;
    }
    return CC1101_Strobe(CC1101_STROBE_SFRX, NULL);
}

void CC1101_AttachSpi(SPI_HandleTypeDef* hspi)
{
    s_hspi = hspi;
}

HAL_StatusTypeDef CC1101_InitForFlipperRemote(void)
{
    static const CC1101_RegPair_t reg_init[] = {
        {CC1101_REG_IOCFG2, SMARTRF_SETTING_IOCFG2},
        {CC1101_REG_IOCFG0, SMARTRF_SETTING_IOCFG0},   /* Asynchronous serial data output. */
        {CC1101_REG_FIFOTHR, SMARTRF_SETTING_FIFOTHR},
        {CC1101_REG_PKTCTRL0, SMARTRF_SETTING_PKTCTRL0}, /* Async serial mode, infinite packet. */
        {CC1101_REG_FSCTRL1, SMARTRF_SETTING_FSCTRL1},
        {CC1101_REG_FREQ2, SMARTRF_SETTING_FREQ2},    /* 433.92 MHz nominal profile. */
        {CC1101_REG_FREQ1, SMARTRF_SETTING_FREQ1},
        {CC1101_REG_FREQ0, SMARTRF_SETTING_FREQ0},
        {CC1101_REG_MDMCFG4, SMARTRF_SETTING_MDMCFG4},
        {CC1101_REG_MDMCFG3, SMARTRF_SETTING_MDMCFG3},
        {CC1101_REG_MDMCFG2, SMARTRF_SETTING_MDMCFG2},  /* ASK/OOK, no sync/preamble detect. */
        {CC1101_REG_DEVIATN, SMARTRF_SETTING_DEVIATN},
        {CC1101_REG_MCSM1, SMARTRF_SETTING_MCSM1},
        {CC1101_REG_MCSM0, SMARTRF_SETTING_MCSM0},
        {CC1101_REG_FOCCFG, SMARTRF_SETTING_FOCCFG},
        {CC1101_REG_BSCFG, SMARTRF_SETTING_BSCFG},
        {CC1101_REG_AGCCTRL2, SMARTRF_SETTING_AGCCTRL2},
        {CC1101_REG_AGCCTRL1, SMARTRF_SETTING_AGCCTRL1},
        {CC1101_REG_AGCCTRL0, SMARTRF_SETTING_AGCCTRL0},
        {CC1101_REG_FREND0, SMARTRF_SETTING_FREND0},
        {CC1101_REG_FSCAL3, SMARTRF_SETTING_FSCAL3},
        {CC1101_REG_FSCAL2, SMARTRF_SETTING_FSCAL2},
        {CC1101_REG_FSCAL1, SMARTRF_SETTING_FSCAL1},
        {CC1101_REG_FSCAL0, SMARTRF_SETTING_FSCAL0},
        {CC1101_REG_TEST2, SMARTRF_SETTING_TEST2},
        {CC1101_REG_TEST1, SMARTRF_SETTING_TEST1},
        {CC1101_REG_TEST0, SMARTRF_SETTING_TEST0}
    };
    uint32_t i;
    HAL_StatusTypeDef hal;

    if (s_hspi == NULL) {
        return HAL_ERROR;
    }

    CC1101_ApplyResetPulseSequence();

    hal = CC1101_Strobe(CC1101_STROBE_SRES, NULL);
    if (hal != HAL_OK) {
        return hal;
    }
    HAL_Delay(1U);

    for (i = 0U; i < (sizeof(reg_init) / sizeof(reg_init[0])); i++) {
        hal = CC1101_WriteReg(reg_init[i].reg, reg_init[i].val);
        if (hal != HAL_OK) {
            return hal;
        }
    }

    hal = CC1101_EnterIdleAndFlushRx();
    if (hal != HAL_OK) {
        return hal;
    }
    hal = CC1101_Strobe(CC1101_STROBE_SFTX, NULL);
    return hal;
}

HAL_StatusTypeDef CC1101_StartRx(void)
{
    HAL_StatusTypeDef hal;

    if (s_hspi == NULL) {
        return HAL_ERROR;
    }

    hal = CC1101_EnterIdleAndFlushRx();
    if (hal != HAL_OK) {
        return hal;
    }
    Princeton_Reset(&s_princeton);
    s_princeton.has_prev_edge = 0U;
    return CC1101_Strobe(CC1101_STROBE_SRX, NULL);
}

HAL_StatusTypeDef CC1101_FeedEdge(uint8_t level, uint32_t timestamp_us, CC1101_Command_t* out_cmd)
{
    uint32_t dt_us;
    uint8_t prior_level;
    uint8_t decoded_bit = 0U;
    uint32_t code_direct;
    uint32_t code_rev;
    CC1101_Command_t mapped;

    *out_cmd = CC1101_CMD_NONE;

    if (!s_princeton.has_prev_edge) {
        /* First edge only establishes baseline timestamp/level. */
        s_princeton.has_prev_edge = 1U;
        s_princeton.prev_level = level;
        s_princeton.prev_ts_us = timestamp_us;
        return HAL_BUSY;
    }

    dt_us = timestamp_us - s_princeton.prev_ts_us;
    s_princeton.prev_ts_us = timestamp_us;
    prior_level = s_princeton.prev_level;

    if (dt_us >= PRINCETON_SYNC_GAP_MIN_US) {
        /* Big idle gap marks frame boundary; start assembling a fresh frame. */
        Princeton_Reset(&s_princeton);
        s_princeton.prev_level = level;
        return HAL_BUSY;
    }

    if (prior_level != 0U) {
        s_princeton.high_us = dt_us;
    } else {
        s_princeton.low_us = dt_us;
    }

    if ((s_princeton.high_us != 0U) &&
        (s_princeton.low_us != 0U) &&
        Princeton_DecodeSymbol(&s_princeton, &decoded_bit)) {
        s_princeton.assembled_code <<= 1U;
        s_princeton.assembled_code |= decoded_bit;
        s_princeton.bit_count++;
        s_princeton.high_us = 0U;
        s_princeton.low_us = 0U;

        if (s_princeton.bit_count >= PRINCETON_BITS) {
            code_direct = s_princeton.assembled_code & 0xFFFFFFUL;
            code_rev = Princeton_Reverse24(code_direct);
            mapped = CC1101_MapCodeToCommand(code_direct);
            if (mapped == CC1101_CMD_NONE) {
                /* Many remotes transmit LSB-first, so try reversed bit order too. */
                mapped = CC1101_MapCodeToCommand(code_rev);
            }
            Princeton_Reset(&s_princeton);
            *out_cmd = mapped;
            s_princeton.prev_level = level;
            return HAL_OK;
        }
    } else if ((s_princeton.high_us != 0U) && (s_princeton.low_us != 0U)) {
        Princeton_Reset(&s_princeton);
    }

    s_princeton.prev_level = level;
    return HAL_BUSY;
}

const char* CC1101_CommandToString(CC1101_Command_t cmd)
{
    switch (cmd) {
        case CC1101_CMD_FORWARD:
            return "FORWARD";
        case CC1101_CMD_BACKWARD:
            return "BACKWARD";
        case CC1101_CMD_LEFT:
            return "LEFT";
        case CC1101_CMD_RIGHT:
            return "RIGHT";
        case CC1101_CMD_CENTER:
            return "CENTER";
        case CC1101_CMD_STOP:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}
