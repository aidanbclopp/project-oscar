#ifndef ROBOT_APP_TUNING_H
#define ROBOT_APP_TUNING_H

/*
 * Field tuning notes:
 * - Weaving on straight runs: lower APP_HEADING_KP or raise APP_HEADING_KD.
 * - Slow heading correction: raise APP_HEADING_KP.
 * - Oscillation around heading: lower APP_HEADING_KP and/or raise APP_HEADING_KD.
 * - Corrections too weak/strong: adjust APP_HEADING_CORR_MAX.
 * - Move distance off: tune APP_MOVE_FORWARD_SCALE / APP_MOVE_BACKWARD_SCALE.
 * - Turn endpoint off: tune APP_HEADING_TOL_DEG and APP_ROTATE_KP.
 * - Turn stalls near target: raise APP_ROTATE_MIN_SPEED.
 * - Frequent motion timeout: raise APP_HEADING_TIMEOUT_MS.
 * - Noisy/missed radio edges: tune APP_RADIO_EDGE_MIN_DELTA_US and APP_RADIO_EDGE_QUEUE_SIZE.
 */

/* ---------------- IMU / heading-hold loop ---------------- */
#define APP_IMU_STARTUP_GYRO_TIMEOUT_MS 3000U  /* Boot wait for gyro calibration before encoder-only fallback. */
#define APP_IMU_CALIB_SAMPLE_MS         100U   /* Poll period while waiting for startup calibration. */
#define APP_IMU_STARTUP_MIN_GYRO_CALIB  3U     /* Startup gyro calibration target (0-3). */

/* Local heading-loop overrides */
#define APP_HEADING_KP             90.0f   /* Straight-line heading proportional gain (aggressive test). */
#define APP_HEADING_KD             16.0f   /* Damping term for heading corrections (aggressive test). */
#define APP_HEADING_CORR_MAX       950.0f  /* Max steering correction mixed into motor commands (aggressive test). */
#define APP_ROTATE_KP              10.0f   /* Turn-rate proportional gain for rotate moves. */
#define APP_ROTATE_MIN_SPEED       220     /* Minimum turn command to keep the robot moving. */
#define APP_HEADING_TOL_DEG        2.0f    /* Turn completion window in degrees. */
#define APP_HEADING_SAMPLE_MS      5U      /* IMU control update period for the motion runner (aggressive test). */
#define APP_HEADING_TIMEOUT_MS     15000U  /* Max time allowed for heading-based motion. */
#define APP_ROTATE_CW_DEG_SCALE    1.10f   /* CW command scale (CW was under-rotating). */
#define APP_ROTATE_CCW_DEG_SCALE   1.00f   /* CCW command scale. */

/* ---------------- Motion distance tuning ---------------- */
#define APP_MOVE_FORWARD_SCALE     1.0f  /* Forward distance trim. */
#define APP_MOVE_BACKWARD_SCALE    1.0f  /* Reverse distance trim. */

/* ---------------- Radio command behavior ---------------- */
#define APP_RADIO_COMMAND_SPEED         800     /* Drive speed for remote commands. */
#define APP_RADIO_COMMAND_MOVE_INCHES   12.0f   /* Step distance for FORWARD/BACKWARD. */
#define APP_RADIO_COMMAND_TURN_DEG      90.0f   /* Turn step for LEFT/RIGHT. */
#define APP_RADIO_COMMAND_CENTER_DEG   -360.0f  /* Turn amount for CENTER. */

#define APP_USER_ROUTINE_MOVE_INCHES    24.0f   /* Distance per straight segment in button routine. */
#define APP_USER_ROUTINE_TURN_DEG       180.0f  /* Turn amount per rotate segment in button routine. */

#define APP_RADIO_CMD_LOG_PERIOD_MS      250U   /* Minimum interval between repeated command logs. */
#define APP_RADIO_OVERFLOW_LOG_PERIOD_MS 1000U  /* Minimum interval between queue-overflow logs. */

/* ---------------- Loop + LED behavior ---------------- */
#define APP_MAIN_LED_BLINK_PERIOD_MS     500U   /* Blink period for board heartbeat LED. */
#define APP_VIEW_REFRESH_MS              100U   /* Refresh interval for runtime log view buffers. */
#define APP_IMU_POLL_MS                  500U   /* Background IMU readiness/calibration poll interval. */
#define APP_ENC_LED_TOGGLE_TICKS         1500U  /* Encoder ticks needed before encoder LED toggles. */

/* ---------------- Radio edge queue / decode feed ---------------- */
#define APP_RADIO_EDGE_QUEUE_SIZE        512U  /* Number of captured GDO0 edges buffered from ISR. */
#define APP_RADIO_EDGE_MIN_DELTA_US      140U  /* Ignore edges shorter than this to filter noise/glitches. */
#define APP_RADIO_EDGE_TICK_EVERY        32U   /* Extra motion-runner ticks while draining dense edge bursts. */

#endif /* ROBOT_APP_TUNING_H */
