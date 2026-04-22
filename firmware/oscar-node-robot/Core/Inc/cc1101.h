#ifndef __CC1101_H__
#define __CC1101_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

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
HAL_StatusTypeDef CC1101_FeedEdge(uint8_t level, uint32_t timestamp_us, CC1101_Command_t* out_cmd);
CC1101_Command_t CC1101_MapCodeToCommand(uint32_t code24);
const char* CC1101_CommandToString(CC1101_Command_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* __CC1101_H__ */
