#include "cc1101.h"

#include "main.h"
#include <stddef.h>

#define CC1101_REG_IOCFG2   0x00U
#define CC1101_REG_IOCFG0   0x02U
#define CC1101_REG_FIFOTHR  0x03U
#define CC1101_REG_PKTCTRL0 0x08U
#define CC1101_REG_FSCTRL1  0x0BU
#define CC1101_REG_FREQ2    0x0DU
#define CC1101_REG_FREQ1    0x0EU
#define CC1101_REG_FREQ0    0x0FU
#define CC1101_REG_MDMCFG4  0x10U
#define CC1101_REG_MDMCFG3  0x11U
#define CC1101_REG_MDMCFG2  0x12U
#define CC1101_REG_DEVIATN  0x15U
#define CC1101_REG_MCSM1    0x17U
#define CC1101_REG_MCSM0    0x18U
#define CC1101_REG_FOCCFG   0x19U
#define CC1101_REG_BSCFG    0x1AU
#define CC1101_REG_AGCCTRL2 0x1BU
#define CC1101_REG_AGCCTRL1 0x1CU
#define CC1101_REG_AGCCTRL0 0x1DU
#define CC1101_REG_FREND0   0x22U
#define CC1101_REG_FSCAL3   0x23U
#define CC1101_REG_FSCAL2   0x24U
#define CC1101_REG_FSCAL1   0x25U
#define CC1101_REG_FSCAL0   0x26U
#define CC1101_REG_TEST2    0x2CU
#define CC1101_REG_TEST1    0x2DU
#define CC1101_REG_TEST0    0x2EU

#define CC1101_STROBE_SRES  0x30U
#define CC1101_STROBE_SRX   0x34U
#define CC1101_STROBE_SIDLE 0x36U
#define CC1101_STROBE_SFRX  0x3AU
#define CC1101_STROBE_SFTX  0x3BU

#define CC1101_SPI_TIMEOUT_MS     20U
#define CC1101_MISO_WAIT_TIMEOUT  10U

#define PRINCETON_BITS                 24U
#define PRINCETON_SHORT_MIN_US         150U
#define PRINCETON_SHORT_MAX_US         750U
#define PRINCETON_LONG_MIN_US          750U
#define PRINCETON_LONG_MAX_US          1650U
#define PRINCETON_SYNC_GAP_MIN_US      8000U

/* These match the .sub files proposed in chat. */
#define FLIPPER_CODE_FORWARD_24   0xC30111UL
#define FLIPPER_CODE_BACKWARD_24  0xC30222UL
#define FLIPPER_CODE_LEFT_24      0xC30333UL
#define FLIPPER_CODE_RIGHT_24     0xC30444UL
#define FLIPPER_CODE_STOP_24      0xC30FFFUL

typedef struct {
    uint8_t reg;
    uint8_t val;
} CC1101_RegPair_t;

static SPI_HandleTypeDef* s_hspi = NULL;

typedef struct {
    uint8_t has_prev_edge;
    uint8_t prev_level;
    uint8_t has_high_low;
    uint8_t bit_count;
    uint32_t prev_ts_us;
    uint32_t high_us;
    uint32_t low_us;
    uint32_t assembled_code;
} PrincetonDecoderState_t;

static PrincetonDecoderState_t s_princeton = {0};

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
    st->has_high_low = 0U;
    st->bit_count = 0U;
    st->assembled_code = 0U;
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
    if (!st->has_high_low) {
        return 0U;
    }

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

void CC1101_AttachSpi(SPI_HandleTypeDef* hspi)
{
    s_hspi = hspi;
}

HAL_StatusTypeDef CC1101_InitForFlipperRemote(void)
{
    static const CC1101_RegPair_t reg_init[] = {
        {CC1101_REG_IOCFG2, 0x2EU},
        {CC1101_REG_IOCFG0, 0x0DU},   /* Asynchronous serial data output. */
        {CC1101_REG_FIFOTHR, 0x47U},
        {CC1101_REG_PKTCTRL0, 0x32U}, /* Async serial mode, infinite packet. */
        {CC1101_REG_FSCTRL1, 0x06U},
        {CC1101_REG_FREQ2, 0x10U},    /* 433.92 MHz nominal profile. */
        {CC1101_REG_FREQ1, 0xB0U},
        {CC1101_REG_FREQ0, 0x71U},
        {CC1101_REG_MDMCFG4, 0xF5U},
        {CC1101_REG_MDMCFG3, 0xB3U},
        {CC1101_REG_MDMCFG2, 0x30U},  /* ASK/OOK, no sync/preamble detect. */
        {CC1101_REG_DEVIATN, 0x15U},
        {CC1101_REG_MCSM1, 0x3CU},
        {CC1101_REG_MCSM0, 0x18U},
        {CC1101_REG_FOCCFG, 0x16U},
        {CC1101_REG_BSCFG, 0x6CU},
        {CC1101_REG_AGCCTRL2, 0x43U},
        {CC1101_REG_AGCCTRL1, 0x40U},
        {CC1101_REG_AGCCTRL0, 0x91U},
        {CC1101_REG_FREND0, 0x11U},
        {CC1101_REG_FSCAL3, 0xE9U},
        {CC1101_REG_FSCAL2, 0x2AU},
        {CC1101_REG_FSCAL1, 0x00U},
        {CC1101_REG_FSCAL0, 0x1FU},
        {CC1101_REG_TEST2, 0x81U},
        {CC1101_REG_TEST1, 0x35U},
        {CC1101_REG_TEST0, 0x09U}
    };
    uint32_t i;
    HAL_StatusTypeDef hal;

    if (s_hspi == NULL) {
        return HAL_ERROR;
    }

    CC1101_Deselect();
    HAL_Delay(1U);
    CC1101_Select();
    HAL_Delay(1U);
    CC1101_Deselect();
    HAL_Delay(1U);

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

    hal = CC1101_Strobe(CC1101_STROBE_SIDLE, NULL);
    if (hal != HAL_OK) {
        return hal;
    }
    hal = CC1101_Strobe(CC1101_STROBE_SFRX, NULL);
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

    hal = CC1101_Strobe(CC1101_STROBE_SIDLE, NULL);
    if (hal != HAL_OK) {
        return hal;
    }
    hal = CC1101_Strobe(CC1101_STROBE_SFRX, NULL);
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

    if (out_cmd == NULL) {
        return HAL_ERROR;
    }
    *out_cmd = CC1101_CMD_NONE;

    if (!s_princeton.has_prev_edge) {
        s_princeton.has_prev_edge = 1U;
        s_princeton.prev_level = level;
        s_princeton.prev_ts_us = timestamp_us;
        return HAL_BUSY;
    }

    dt_us = timestamp_us - s_princeton.prev_ts_us;
    s_princeton.prev_ts_us = timestamp_us;
    prior_level = s_princeton.prev_level;

    if (dt_us >= PRINCETON_SYNC_GAP_MIN_US) {
        Princeton_Reset(&s_princeton);
    }

    if (prior_level != 0U) {
        s_princeton.high_us = dt_us;
    } else {
        s_princeton.low_us = dt_us;
    }

    if ((s_princeton.high_us != 0U) && (s_princeton.low_us != 0U)) {
        s_princeton.has_high_low = 1U;
    }

    if (Princeton_DecodeSymbol(&s_princeton, &decoded_bit)) {
        s_princeton.assembled_code <<= 1U;
        s_princeton.assembled_code |= decoded_bit;
        s_princeton.bit_count++;
        s_princeton.high_us = 0U;
        s_princeton.low_us = 0U;
        s_princeton.has_high_low = 0U;

        if (s_princeton.bit_count >= PRINCETON_BITS) {
            code_direct = s_princeton.assembled_code & 0xFFFFFFUL;
            code_rev = Princeton_Reverse24(code_direct);
            mapped = CC1101_MapCodeToCommand(code_direct);
            if (mapped == CC1101_CMD_NONE) {
                mapped = CC1101_MapCodeToCommand(code_rev);
            }
            Princeton_Reset(&s_princeton);
            *out_cmd = mapped;
            s_princeton.prev_level = level;
            return HAL_OK;
        }
    } else if (s_princeton.has_high_low) {
        Princeton_Reset(&s_princeton);
    }

    s_princeton.prev_level = level;
    return HAL_BUSY;
}

CC1101_Command_t CC1101_MapCodeToCommand(uint32_t code24)
{
    switch (code24 & 0xFFFFFFUL) {
        case FLIPPER_CODE_FORWARD_24:
            return CC1101_CMD_FORWARD;
        case FLIPPER_CODE_BACKWARD_24:
            return CC1101_CMD_BACKWARD;
        case FLIPPER_CODE_LEFT_24:
            return CC1101_CMD_LEFT;
        case FLIPPER_CODE_RIGHT_24:
            return CC1101_CMD_RIGHT;
        case FLIPPER_CODE_STOP_24:
            return CC1101_CMD_STOP;
        default:
            return CC1101_CMD_NONE;
    }
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
        case CC1101_CMD_STOP:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}
