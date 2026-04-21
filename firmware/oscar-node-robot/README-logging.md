# Persistent Logging (STM32F401CCU6)

This firmware supports persistent on-chip logging to internal flash so logs can be inspected after disconnecting and reconnecting the debugger.

## Compile Flag

- CMake option: `OSCAR_ENABLE_PERSISTENT_LOGS`
- Default: `ON`

Configure examples:

- Enabled (default):
  - `cmake -S . -B build/Debug`
- Disabled:
  - `cmake -S . -B build/Debug-no-log -DOSCAR_ENABLE_PERSISTENT_LOGS=OFF`

When disabled:

- Firmware still builds and runs.
- Semihosting `printf` behavior remains unchanged when debugger is attached.
- Persistent log storage and debug views are compiled as safe no-ops.

## Fileset Layout

- Flash backend:
  - `Core/Inc/log_store.h`
  - `Core/Src/log_store.c`
- Runtime/debug integration:
  - `Core/Inc/log_runtime.h`
  - `Core/Src/log_runtime.c`
- Call sites:
  - `Core/Src/syscalls.c` (routes `_write` into `LogRuntime_Write`)
  - `Core/Src/main.c` (calls `LogRuntime_Init` and periodic `LogRuntime_RefreshViews`)

## Flash Behavior

- Logs are written into reserved `LOGFLASH` region in internal flash.
- Records are appended with basic integrity metadata.
- When full, the region erases and wraps (oldest retained history is lost at wrap).
- Writes from ISR context are dropped by the backend for safety.

## Debugger Usage (VS Code + ST-Link GDB Server)

1. Start a debug session and pause target.
2. In Watch, add:
   - `g_log_record_count`
   - `g_log_dropped_writes`
   - `g_log_write_addr`
   - `g_log_last_string`
   - `g_log_view_count`
   - `g_log_view_start_index`
   - `g_log_view_strings[0]`
   - `g_log_view_strings[1]`
3. Resume briefly and pause again to refresh view buffers.

Notes:

- `g_log_last_string` is the newest record as a printable C string.
- `g_log_view_strings[i]` contains a window of recent records.
- `g_log_view_start_index + i` gives the absolute record index for each displayed slot.

## Typical Workflow

1. Flash firmware.
2. Disconnect debugger and run robot.
3. Reconnect debugger later.
4. Inspect `g_log_last_string` and `g_log_view_strings[i]`.

This is expected to work across debugger disconnect/reconnect and power cycles, until the log region wraps and erases.
