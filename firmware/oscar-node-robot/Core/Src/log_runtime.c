#include "log_runtime.h"

#include "log_store.h"

volatile uint32_t g_log_record_count = 0U;
volatile uint32_t g_log_dropped_writes = 0U;
volatile uint32_t g_log_write_addr = 0U;
volatile uint16_t g_log_last_len = 0U;
volatile uint8_t g_log_last_record[256];
volatile char g_log_last_string[257];
__attribute__((used)) volatile uint32_t g_log_view_count = 0U;
__attribute__((used)) volatile uint32_t g_log_view_start_index = 0U;
__attribute__((used)) volatile char g_log_view_strings[16][257];

#if OSCAR_ENABLE_PERSISTENT_LOGS
static void LogRuntime_CopyPrintableString(volatile char *dst, const uint8_t *src, uint16_t len, uint16_t dst_size)
{
  uint16_t i;
  uint16_t copy_len = len;

  if (copy_len >= dst_size) {
    copy_len = (uint16_t)(dst_size - 1U);
  }

  for (i = 0U; i < copy_len; i++) {
    char c = (char)src[i];
    dst[i] = (((c >= 32) && (c <= 126)) || c == '\r' || c == '\n' || c == '\t') ? c : '.';
  }
  dst[copy_len] = '\0';
}

static void LogRuntime_UpdateSnapshot(void)
{
  const LogStoreDebugState_t *state = LogStore_GetDebugState();
  uint32_t count = LogStore_GetRecordCount();

  g_log_record_count = count;
  g_log_dropped_writes = state->dropped_writes;
  g_log_write_addr = state->write_addr;

  if (count > 0U) {
    if (LogStore_ReadRecord(count - 1U, (uint8_t *)g_log_last_record, sizeof(g_log_last_record), (uint16_t *)&g_log_last_len) != LOG_STORE_OK) {
      g_log_last_len = 0U;
      g_log_last_string[0] = '\0';
    } else {
      LogRuntime_CopyPrintableString(g_log_last_string, (const uint8_t *)g_log_last_record, g_log_last_len, sizeof(g_log_last_string));
    }
  } else {
    g_log_last_len = 0U;
    g_log_last_string[0] = '\0';
  }
}

void LogRuntime_Init(void)
{
  LogStore_Init();
  LogRuntime_UpdateSnapshot();
}

void LogRuntime_Write(const uint8_t *data, uint16_t len)
{
  static uint8_t log_line_buf[256];
  static uint16_t log_line_len = 0U;
  uint16_t i;

  if (data == NULL || len == 0U) {
    return;
  }

  for (i = 0U; i < len; i++) {
    uint8_t c = data[i];
    log_line_buf[log_line_len++] = c;

    if ((c == '\n') || (log_line_len == sizeof(log_line_buf))) {
      (void)LogStore_Append(log_line_buf, log_line_len);
      log_line_len = 0U;
    }
  }
}

void LogRuntime_RefreshViews(uint32_t max_records)
{
  uint32_t total;
  uint32_t capacity = sizeof(g_log_view_strings) / sizeof(g_log_view_strings[0]);
  uint32_t count = max_records;
  uint32_t start;
  uint32_t slot;

  LogRuntime_UpdateSnapshot();

  total = LogStore_GetRecordCount();
  if (count == 0U || count > capacity) {
    count = capacity;
  }
  if (total < count) {
    count = total;
  }

  g_log_view_count = count;
  if (count == 0U) {
    g_log_view_start_index = 0U;
    return;
  }

  start = total - count;
  g_log_view_start_index = start;

  for (slot = 0U; slot < count; slot++) {
    uint16_t len = 0U;
    if (LogStore_ReadRecord(start + slot, (uint8_t *)g_log_last_record, sizeof(g_log_last_record), &len) != LOG_STORE_OK) {
      g_log_view_strings[slot][0] = '\0';
    } else {
      LogRuntime_CopyPrintableString(g_log_view_strings[slot], (const uint8_t *)g_log_last_record, len, sizeof(g_log_view_strings[slot]));
    }
  }
}
#else
void LogRuntime_Init(void)
{
  g_log_record_count = 0U;
  g_log_dropped_writes = 0U;
  g_log_write_addr = 0U;
  g_log_last_len = 0U;
  g_log_last_string[0] = '\0';
  g_log_view_count = 0U;
  g_log_view_start_index = 0U;
  g_log_view_strings[0][0] = '\0';
}

void LogRuntime_Write(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
}

void LogRuntime_RefreshViews(uint32_t max_records)
{
  (void)max_records;
  g_log_record_count = 0U;
  g_log_dropped_writes = 0U;
  g_log_write_addr = 0U;
  g_log_last_len = 0U;
  g_log_last_string[0] = '\0';
  g_log_view_count = 0U;
  g_log_view_start_index = 0U;
  g_log_view_strings[0][0] = '\0';
}
#endif
