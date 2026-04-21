#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
  LOG_STORE_OK = 0,
  LOG_STORE_ERR_PARAM = -1,
  LOG_STORE_ERR_ISR_CONTEXT = -2,
  LOG_STORE_ERR_FLASH = -3,
  LOG_STORE_ERR_CORRUPT = -4
} LogStoreStatus_t;

typedef struct
{
  uint32_t start_addr;
  uint32_t end_addr;
  uint32_t write_addr;
  uint32_t record_count;
  uint32_t dropped_writes;
  uint8_t initialized;
} LogStoreDebugState_t;

void LogStore_Init(void);
LogStoreStatus_t LogStore_Append(const uint8_t *data, uint16_t len);
LogStoreStatus_t LogStore_Clear(void);
LogStoreStatus_t LogStore_ReadRecord(uint32_t index, uint8_t *out, uint16_t out_len, uint16_t *actual_len);
uint32_t LogStore_GetRecordCount(void);
const LogStoreDebugState_t *LogStore_GetDebugState(void);

#endif /* LOG_STORE_H */
