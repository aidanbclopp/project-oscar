#ifndef LOG_RUNTIME_H
#define LOG_RUNTIME_H

#include <stdint.h>

#define LOG_RUNTIME_RECORD_MAX_BYTES 256U
#define LOG_RUNTIME_VIEW_SLOTS       16U
#define LOG_RUNTIME_STRING_BYTES     (LOG_RUNTIME_RECORD_MAX_BYTES + 1U)

extern volatile uint32_t g_log_record_count;
extern volatile uint32_t g_log_dropped_writes;
extern volatile uint32_t g_log_write_addr;
extern volatile uint16_t g_log_last_len;
extern volatile uint8_t g_log_last_record[LOG_RUNTIME_RECORD_MAX_BYTES];
extern volatile char g_log_last_string[LOG_RUNTIME_STRING_BYTES];
extern volatile uint32_t g_log_view_count;
extern volatile uint32_t g_log_view_start_index;
extern volatile char g_log_view_strings[LOG_RUNTIME_VIEW_SLOTS][LOG_RUNTIME_STRING_BYTES];

void LogRuntime_Init(void);
void LogRuntime_Write(const uint8_t *data, uint16_t len);
void LogRuntime_RefreshViews(uint32_t max_records);

#endif /* LOG_RUNTIME_H */
