#ifndef LOG_RUNTIME_H
#define LOG_RUNTIME_H

#include <stdint.h>

extern volatile uint32_t g_log_record_count;
extern volatile uint32_t g_log_dropped_writes;
extern volatile uint32_t g_log_write_addr;
extern volatile uint16_t g_log_last_len;
extern volatile uint8_t g_log_last_record[256];
extern volatile char g_log_last_string[257];
extern volatile uint32_t g_log_view_count;
extern volatile uint32_t g_log_view_start_index;
extern volatile char g_log_view_strings[16][257];

void LogRuntime_Init(void);
void LogRuntime_Write(const uint8_t *data, uint16_t len);
void LogRuntime_RefreshViews(uint32_t max_records);

#endif /* LOG_RUNTIME_H */
