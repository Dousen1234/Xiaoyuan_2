#ifndef CLAW_CONFIG_H
#define CLAW_CONFIG_H

#define CLAW_MOTOR_COUNT                       3U
#define CLAW_MOTOR_POLE_PAIRS                  1U
#define CLAW_TOTAL_GEAR_RATIO_X1000            628500U
#define CLAW_PWM_PERIOD_TICKS                  3600U

#define CLAW_CONTROL_PERIOD_MS                 10U
#define CLAW_DIRECTION_SETTLE_MS               20U
#define CLAW_DIRECTION_QUIET_MS                300U
/* Uncalibrated single-channel pulse; independent of PID profiles. */
#define CLAW_JOG_DUTY_PERMILLE                  200U
#define CLAW_RUN_IN_DUTY_PERMILLE              1000U
#define CLAW_JOG_DURATION_MS                    200U
#define CLAW_SEEK_DUTY_PERMILLE                 900U
#define CLAW_SEEK_NO_PROGRESS_MS                500U
#define CLAW_SEEK_TIMEOUT_MS                    5000U
#define CLAW_JOG_MAX_COUNTS                     10L
/* Temporary direction observation: physical module 6, raw DIR=0 only. */
#define CLAW_JOG_M6_DIR0_MAX_COUNTS             30L
#define CLAW_COMMAND_WATCHDOG_MS               500U
#define CLAW_HOLD_CLD_RESET_PERIOD_MS         1000U
#define CLAW_HOLD_CLD_RESET_PULSE_MS             20U
#define CLAW_STALL_TIMEOUT_MS                  800U
#define CLAW_MOVE_TIMEOUT_MS                   15000U

#define CLAW_PWM_SAFE_MAX_PERMILLE            1000U
#define CLAW_POSITION_DEADBAND_COUNTS           1L
/* Initial claw commissioning: allow gearbox/underactuated speed mismatch
 * while retaining the per-motor 800 ms no-Hall stall protection. */
#define CLAW_SYNC_FAULT_COUNTS                 150L
#define CLAW_SYNC_FAULT_TIME_MS                2000U

#define CLAW_MIN_TARGET_MDEG                   0L
#define CLAW_MAX_TARGET_MDEG                   120000L
/* Before the user confirms the physical open limit and sends zero, allow
 * only a conservative negative commissioning range. */
#define CLAW_PREZERO_MIN_TARGET_MDEG           (-30000L)
#define CLAW_DEFAULT_SPEED_DRPM                40U
#define CLAW_MIN_SPEED_DRPM                    40U
#define CLAW_MAX_SPEED_DRPM                   170U

#define CLAW_POSITION_FEEDBACK_PERIOD_MS       20U
#define CLAW_STATUS_FEEDBACK_PERIOD_MS         100U
#define CLAW_DIAGNOSTIC_FEEDBACK_PERIOD_MS     20U

#define CLAW_HALL_SEQUENCE_0                   0x1U
#define CLAW_HALL_SEQUENCE_1                   0x5U
#define CLAW_HALL_SEQUENCE_2                   0x4U
#define CLAW_HALL_SEQUENCE_3                   0x6U
#define CLAW_HALL_SEQUENCE_4                   0x2U
#define CLAW_HALL_SEQUENCE_5                   0x3U

/* Group 2 direction/Hall signs verified by bidirectional jogs.
 * Enable only confirmed local channels (bit0=M1, bit1=M2, bit2=M3).
 * Use zero for CAN-only work; mask=7 enables all three confirmed local channels. */
#ifndef CLAW_COMMISSIONED_MASK
#define CLAW_COMMISSIONED_MASK                7U
#endif
/* Zero disables the temporary step bound; protocol absolute bounds remain. */
#ifndef CLAW_TRIAL_MAX_STEP_MDEG
#define CLAW_TRIAL_MAX_STEP_MDEG              0L
#endif
static const unsigned char claw_direction_high[3] = {0U, 0U, 0U};
#define CLAW_M1_HALL_COUNT_SIGN                1
#define CLAW_M2_HALL_COUNT_SIGN                1
#define CLAW_M3_HALL_COUNT_SIGN                1
/* Tested driver buzzes/holds when active-low BRAK is asserted. */
#define CLAW_ACTIVE_BRAKE_ENABLED              0U

#endif /* CLAW_CONFIG_H */
