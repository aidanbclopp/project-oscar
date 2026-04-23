#ifndef __CC1101_H__
#define __CC1101_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* CC1101 register addresses used by this driver. */
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

/* CC1101 command strobes used by this driver. */
#define CC1101_STROBE_SRES  0x30U
#define CC1101_STROBE_SRX   0x34U
#define CC1101_STROBE_SIDLE 0x36U
#define CC1101_STROBE_SFRX  0x3AU
#define CC1101_STROBE_SFTX  0x3BU

typedef enum {
    CC1101_CMD_NONE = 0,
    CC1101_CMD_FORWARD,
    CC1101_CMD_BACKWARD,
    CC1101_CMD_LEFT,
    CC1101_CMD_RIGHT,
    CC1101_CMD_CENTER,
    CC1101_CMD_STOP
} CC1101_Command_t;

void CC1101_AttachSpi(SPI_HandleTypeDef* hspi);
HAL_StatusTypeDef CC1101_InitForFlipperRemote(void);
HAL_StatusTypeDef CC1101_StartRx(void);
/* Returns HAL_BUSY while assembling a frame and HAL_OK when a frame completes. */
HAL_StatusTypeDef CC1101_FeedEdge(uint8_t level, uint32_t timestamp_us, CC1101_Command_t* out_cmd);
const char* CC1101_CommandToString(CC1101_Command_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* __CC1101_H__ */
