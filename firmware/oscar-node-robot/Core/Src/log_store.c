#include "log_store.h"

#include "main.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#define LOG_RECORD_MAGIC ((uint16_t)0x4C47U)

typedef struct
{
  uint16_t magic;
  uint16_t len;
  uint32_t checksum;
} LogRecordHeader_t;

extern uint32_t _log_flash_start;
extern uint32_t _log_flash_end;

static LogStoreDebugState_t g_log_state;

static uint32_t LogStore_SectorFromAddress(uint32_t addr)
{
  if (addr < 0x08004000U) return FLASH_SECTOR_0;
  if (addr < 0x08008000U) return FLASH_SECTOR_1;
  if (addr < 0x0800C000U) return FLASH_SECTOR_2;
  if (addr < 0x08010000U) return FLASH_SECTOR_3;
  if (addr < 0x08020000U) return FLASH_SECTOR_4;
  if (addr < 0x08040000U) return FLASH_SECTOR_5;
  return 0xFFFFFFFFU;
}

static uint8_t LogStore_InIsr(void)
{
  return ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U) ? 1U : 0U;
}

static uint32_t LogStore_Checksum(const uint8_t *data, uint16_t len)
{
  uint32_t sum = 2166136261u;
  uint16_t i;
  for (i = 0U; i < len; i++) {
    sum ^= (uint32_t)data[i];
    sum *= 16777619u;
  }
  return sum;
}

static uint32_t LogStore_RecordSpan(uint16_t len)
{
  uint32_t total = (uint32_t)sizeof(LogRecordHeader_t) + (uint32_t)len;
  return (total + 3U) & ~0x3U;
}

static LogStoreStatus_t LogStore_EraseRegion(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t error_sector = 0U;
  uint32_t sector = LogStore_SectorFromAddress(g_log_state.start_addr);

  if (sector == 0xFFFFFFFFU) {
    return LOG_STORE_ERR_FLASH;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = sector;
  erase.NbSectors = 1;

  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&erase, &error_sector) != HAL_OK) {
    HAL_FLASH_Lock();
    return LOG_STORE_ERR_FLASH;
  }
  HAL_FLASH_Lock();
  return LOG_STORE_OK;
}

static LogStoreStatus_t LogStore_ProgramBytes(uint32_t addr, const uint8_t *data, uint32_t len)
{
  uint32_t offset = 0U;
  HAL_FLASH_Unlock();
  while (offset < len) {
    uint32_t word = 0xFFFFFFFFU;
    uint32_t chunk = len - offset;
    if (chunk > 4U) {
      chunk = 4U;
    }
    memcpy(&word, &data[offset], chunk);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + offset, word) != HAL_OK) {
      HAL_FLASH_Lock();
      return LOG_STORE_ERR_FLASH;
    }
    offset += 4U;
  }
  HAL_FLASH_Lock();
  return LOG_STORE_OK;
}

static uint8_t LogStore_RecordIsErased(const LogRecordHeader_t *hdr)
{
  return (hdr->magic == 0xFFFFU && hdr->len == 0xFFFFU && hdr->checksum == 0xFFFFFFFFU) ? 1U : 0U;
}

void LogStore_Init(void)
{
  uint32_t addr;
  g_log_state.start_addr = (uint32_t)&_log_flash_start;
  g_log_state.end_addr = (uint32_t)&_log_flash_end;
  g_log_state.write_addr = g_log_state.start_addr;
  g_log_state.record_count = 0U;
  g_log_state.dropped_writes = 0U;
  g_log_state.initialized = 0U;

  if ((g_log_state.end_addr <= g_log_state.start_addr) ||
      ((g_log_state.start_addr & 0x3U) != 0U) ||
      (LogStore_SectorFromAddress(g_log_state.start_addr) == 0xFFFFFFFFU)) {
    return;
  }

  addr = g_log_state.start_addr;
  while ((addr + sizeof(LogRecordHeader_t)) <= g_log_state.end_addr) {
    const LogRecordHeader_t *hdr = (const LogRecordHeader_t *)addr;
    uint32_t span;
    uint32_t payload_addr;
    uint16_t i;
    uint32_t sum = 2166136261u;

    if (LogStore_RecordIsErased(hdr)) {
      break;
    }
    if (hdr->magic != LOG_RECORD_MAGIC) {
      break;
    }

    span = LogStore_RecordSpan(hdr->len);
    if ((addr + span) > g_log_state.end_addr) {
      break;
    }

    payload_addr = addr + sizeof(LogRecordHeader_t);
    for (i = 0U; i < hdr->len; i++) {
      sum ^= (uint32_t)(*(const uint8_t *)(payload_addr + i));
      sum *= 16777619u;
    }
    if (sum != hdr->checksum) {
      break;
    }

    g_log_state.record_count++;
    addr += span;
  }

  g_log_state.write_addr = addr;
  g_log_state.initialized = 1U;
}

LogStoreStatus_t LogStore_Clear(void)
{
  if (LogStore_InIsr()) {
    return LOG_STORE_ERR_ISR_CONTEXT;
  }

  if (LogStore_EraseRegion() != LOG_STORE_OK) {
    return LOG_STORE_ERR_FLASH;
  }

  g_log_state.write_addr = g_log_state.start_addr;
  g_log_state.record_count = 0U;
  return LOG_STORE_OK;
}

LogStoreStatus_t LogStore_Append(const uint8_t *data, uint16_t len)
{
  LogRecordHeader_t hdr;
  uint32_t span;
  uint8_t write_buf[sizeof(LogRecordHeader_t) + 256U];

  if (data == NULL || len == 0U || len > 256U) {
    return LOG_STORE_ERR_PARAM;
  }
  if (!g_log_state.initialized) {
    return LOG_STORE_ERR_PARAM;
  }
  if (LogStore_InIsr()) {
    g_log_state.dropped_writes++;
    return LOG_STORE_ERR_ISR_CONTEXT;
  }

  span = LogStore_RecordSpan(len);
  if ((g_log_state.write_addr + span) > g_log_state.end_addr) {
    if (LogStore_Clear() != LOG_STORE_OK) {
      g_log_state.dropped_writes++;
      return LOG_STORE_ERR_FLASH;
    }
  }

  hdr.magic = LOG_RECORD_MAGIC;
  hdr.len = len;
  hdr.checksum = LogStore_Checksum(data, len);

  memset(write_buf, 0xFF, sizeof(write_buf));
  memcpy(write_buf, &hdr, sizeof(hdr));
  memcpy(&write_buf[sizeof(hdr)], data, len);

  if (LogStore_ProgramBytes(g_log_state.write_addr, write_buf, span) != LOG_STORE_OK) {
    g_log_state.dropped_writes++;
    return LOG_STORE_ERR_FLASH;
  }

  g_log_state.write_addr += span;
  g_log_state.record_count++;
  return LOG_STORE_OK;
}

LogStoreStatus_t LogStore_ReadRecord(uint32_t index, uint8_t *out, uint16_t out_len, uint16_t *actual_len)
{
  uint32_t addr = g_log_state.start_addr;
  uint32_t current = 0U;

  if (!g_log_state.initialized || actual_len == NULL) {
    return LOG_STORE_ERR_PARAM;
  }

  while ((addr + sizeof(LogRecordHeader_t)) <= g_log_state.write_addr) {
    const LogRecordHeader_t *hdr = (const LogRecordHeader_t *)addr;
    uint32_t span;
    uint32_t payload_addr;
    uint32_t sum = 2166136261u;
    uint16_t i;

    if (hdr->magic != LOG_RECORD_MAGIC) {
      return LOG_STORE_ERR_CORRUPT;
    }

    span = LogStore_RecordSpan(hdr->len);
    if (hdr->len == 0U || hdr->len > 256U) {
      return LOG_STORE_ERR_CORRUPT;
    }
    if ((addr + span) > g_log_state.write_addr || (addr + span) > g_log_state.end_addr) {
      return LOG_STORE_ERR_CORRUPT;
    }
    payload_addr = addr + sizeof(LogRecordHeader_t);

    for (i = 0U; i < hdr->len; i++) {
      sum ^= (uint32_t)(*(const uint8_t *)(payload_addr + i));
      sum *= 16777619u;
    }
    if (sum != hdr->checksum) {
      return LOG_STORE_ERR_CORRUPT;
    }

    if (current == index) {
      if (out == NULL || out_len < hdr->len) {
        return LOG_STORE_ERR_PARAM;
      }
      memcpy(out, (const void *)payload_addr, hdr->len);
      *actual_len = hdr->len;
      return LOG_STORE_OK;
    }

    addr += span;
    current++;
  }

  return LOG_STORE_ERR_PARAM;
}

uint32_t LogStore_GetRecordCount(void)
{
  return g_log_state.record_count;
}

const LogStoreDebugState_t *LogStore_GetDebugState(void)
{
  return &g_log_state;
}
