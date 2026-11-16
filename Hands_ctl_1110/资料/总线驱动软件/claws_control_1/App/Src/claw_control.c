#include "claw_control.h"

#include <stddef.h>

#include "claw_config.h"
#include "claw_profiles.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "board.h"
#include "claw_can_config.h"

typedef struct
{
  volatile int32_t position_counts;
  volatile int8_t previous_sector;
  volatile uint8_t hall_code;
  volatile uint8_t invalid_edges;
  volatile uint8_t wrong_direction_edges;
  volatile uint32_t last_hall_tick;
  int32_t start_counts;
  int8_t command_direction;
  uint16_t duty_permille;
} MotorRuntime;

static const uint8_t hall_sequence[6] =
{
  CLAW_HALL_SEQUENCE_0, CLAW_HALL_SEQUENCE_1, CLAW_HALL_SEQUENCE_2,
  CLAW_HALL_SEQUENCE_3, CLAW_HALL_SEQUENCE_4, CLAW_HALL_SEQUENCE_5
};

static const int8_t hall_count_sign[CLAW_MOTOR_COUNT] =
{
  CLAW_M1_HALL_COUNT_SIGN, CLAW_M2_HALL_COUNT_SIGN, CLAW_M3_HALL_COUNT_SIGN
};

static GPIO_TypeDef * const dir_port[CLAW_MOTOR_COUNT] =
{
  M1_DIR_GPIO_Port, M2_DIR_GPIO_Port, M3_DIR_GPIO_Port
};

static const uint16_t dir_pin[CLAW_MOTOR_COUNT] =
{
  M1_DIR_Pin, M2_DIR_Pin, M3_DIR_Pin
};

static GPIO_TypeDef * const brake_port[CLAW_MOTOR_COUNT] =
{
  M1_NBRAKE_GPIO_Port, M2_NBRAKE_GPIO_Port, M3_NBRAKE_GPIO_Port
};

static const uint16_t brake_pin[CLAW_MOTOR_COUNT] =
{
  M1_NBRAKE_Pin, M2_NBRAKE_Pin, M3_NBRAKE_Pin
};

static GPIO_TypeDef * const hall_port[CLAW_MOTOR_COUNT][3] =
{
  {M1_HA_GPIO_Port, M1_HB_GPIO_Port, M1_HC_GPIO_Port},
  {M2_HA_GPIO_Port, M2_HB_GPIO_Port, M2_HC_GPIO_Port},
  {M3_HA_GPIO_Port, M3_HB_GPIO_Port, M3_HC_GPIO_Port}
};

static const uint16_t hall_pin[CLAW_MOTOR_COUNT][3] =
{
  {M1_HA_Pin, M1_HB_Pin, M1_HC_Pin},
  {M2_HA_Pin, M2_HB_Pin, M2_HC_Pin},
  {M3_HA_Pin, M3_HB_Pin, M3_HC_Pin}
};

static volatile MotorRuntime motors[CLAW_MOTOR_COUNT];
static volatile ClawState claw_state;
static volatile uint8_t fault_mask;
static volatile uint8_t last_command_sequence;
static volatile uint8_t last_command_result;
static volatile uint32_t last_command_tick;
static volatile int32_t target_counts;
static volatile bool new_fault_pending;
static bool action_sequence_valid;
static uint8_t last_action_sequence;
static uint32_t move_start_tick;
static uint32_t motion_enable_tick;
static ClawPidState pid_state[CLAW_MOTOR_COUNT];
static bool startup_done[CLAW_MOTOR_COUNT];
static bool motor_done[CLAW_MOTOR_COUNT];
static int32_t path_counts[CLAW_MOTOR_COUNT];
static int32_t longest_path;
static uint32_t trajectory_ms;
static ClawPidConfig runtime_profiles[3];
static uint16_t seek_motor_duty[3];
static uint16_t seek_duty, seek_no_progress_ms, seek_timeout_ms;
static uint16_t grip_duty[3], grip_no_progress_ms, grip_timeout_ms;
static volatile bool grip_active;
static uint16_t hold_duty[3];
static uint32_t hold_cld_reset_tick;
static const ClawPidConfig *profile(uint8_t motor)
{
  return &runtime_profiles[motor];
}
static uint32_t last_control_tick;
static uint32_t sync_fault_start_tick;
static uint8_t diagnostic_counter;
static uint8_t requested_speed_drpm;
static uint8_t active_motor_mask;
static bool zero_calibrated;

static ClawResult record_command_result(ClawResult result)
{
  last_command_result = (uint8_t)result;
  return result;
}

static uint32_t irq_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void irq_unlock(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static int32_t rounded_divide_s64(int64_t numerator, int64_t denominator)
{
  numerator += (numerator >= 0) ? (denominator / 2) : -(denominator / 2);
  return (int32_t)(numerator / denominator);
}

static int32_t abs_i32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static uint8_t command_motor_mask(ClawCommandOpcode opcode)
{
  switch (opcode)
  {
    case CLAW_COMMAND_SET_TARGET:
      return (uint8_t)((1U << CLAW_MOTOR_COUNT) - 1U);
    case CLAW_COMMAND_SET_TARGET_M1:
      return 1U << 0U;
    case CLAW_COMMAND_SET_TARGET_M2:
      return 1U << 1U;
    case CLAW_COMMAND_SET_TARGET_M3:
      return 1U << 2U;
    default:
      return 0U;
  }
}

static uint8_t active_motor_count(uint8_t motor_mask)
{
  uint8_t count = 0U;
  uint8_t motor;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((motor_mask & (uint8_t)(1U << motor)) != 0U)
    {
      ++count;
    }
  }
  return count;
}

static int16_t clamp_i16(int32_t value)
{
  if (value > INT16_MAX)
  {
    return INT16_MAX;
  }
  if (value < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static uint8_t read_hall_code(uint8_t motor)
{
  uint8_t code = 0U;
  uint8_t phase;
  for (phase = 0U; phase < 3U; ++phase)
  {
    if (HAL_GPIO_ReadPin(hall_port[motor][phase], hall_pin[motor][phase]) == GPIO_PIN_SET)
    {
      code |= (uint8_t)(1U << phase);
    }
  }
  return code;
}

static int8_t hall_to_sector(uint8_t code)
{
  uint8_t index;
  for (index = 0U; index < 6U; ++index)
  {
    if (hall_sequence[index] == code)
    {
      return (int8_t)index;
    }
  }
  return -1;
}

static void set_pwm(uint8_t motor, uint16_t duty_permille)
{
  if ((claw_state == CLAW_STATE_FAULT) && (duty_permille > 0U))
  {
    duty_permille = 0U;
  }
  if (duty_permille > CLAW_PWM_SAFE_MAX_PERMILLE)
  {
    duty_permille = CLAW_PWM_SAFE_MAX_PERMILLE;
  }
  motors[motor].duty_permille = Board_SetPWM(motor, duty_permille);
}

static void set_brake(bool enabled)
{
  uint8_t motor;
  if (CLAW_ACTIVE_BRAKE_ENABLED == 0U)
  {
    enabled = false;
  }
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    HAL_GPIO_WritePin(brake_port[motor], brake_pin[motor],
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
}

static void set_direction(int8_t direction, uint8_t motor_mask)
{
  uint8_t motor;

  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((motor_mask & (uint8_t)(1U << motor)) != 0U)
    {
      bool positive_high = claw_direction_high[motor] != 0U;
      bool high = direction >= 0 ? positive_high : !positive_high;
      HAL_GPIO_WritePin(dir_port[motor], dir_pin[motor], high ? GPIO_PIN_SET : GPIO_PIN_RESET);
      motors[motor].command_direction = direction;
    }
  }
}

static bool jog_active;
static bool run_in_active;
static bool seek_active;
static uint8_t seek_remaining_mask;
static uint8_t jog_motor;
static int32_t jog_limit_counts;
static uint32_t jog_stop_tick;

static void stop_all(bool brake_enabled)
{
  uint8_t motor;
  if (jog_active || run_in_active || seek_active || grip_active) jog_stop_tick = HAL_GetTick();
  jog_active = false;
  run_in_active = false;
  seek_active = false;
  grip_active = false;
  hold_cld_reset_tick = HAL_GetTick();
  seek_remaining_mask = 0U;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    set_pwm(motor, 0U);
    motors[motor].command_direction = 0;
  }
  set_brake(brake_enabled);
}

static void latch_fault(uint8_t mask)
{
  fault_mask |= mask;
  claw_state = CLAW_STATE_FAULT;
  new_fault_pending = true;
  stop_all(true);
}

int32_t ClawControl_AngleMilliDegToCounts(int32_t angle_mdeg)
{
  int64_t numerator = (int64_t)angle_mdeg * 6LL * CLAW_MOTOR_POLE_PAIRS
                    * CLAW_TOTAL_GEAR_RATIO_X1000;
  return rounded_divide_s64(numerator, 360000000LL);
}

int32_t ClawControl_CountsToAngleMilliDeg(int32_t counts)
{
  int64_t denominator = 6LL * CLAW_MOTOR_POLE_PAIRS * CLAW_TOTAL_GEAR_RATIO_X1000;
  return rounded_divide_s64((int64_t)counts * 360000000LL, denominator);
}

void ClawControl_Init(void)
{
  uint8_t motor;
  uint32_t now = HAL_GetTick();

  seek_duty = CLAW_SEEK_DUTY_PERMILLE;
  seek_motor_duty[0] = 900U;  /* module 4 */
  seek_motor_duty[1] = 1000U; /* module 5 */
  seek_motor_duty[2] = 900U;  /* module 6 */
  seek_no_progress_ms = CLAW_SEEK_NO_PROGRESS_MS;
  seek_timeout_ms = CLAW_SEEK_TIMEOUT_MS;
  grip_no_progress_ms = 500U;
  grip_timeout_ms = 5000U;
  grip_duty[0] = 900U; /* module 4 */
  grip_duty[1] = 965U; /* module 5 */
  grip_duty[2] = 900U; /* module 6 */
  hold_duty[0] = 950U; /* module 4 */
  hold_duty[1] = 950U; /* module 5 */
  hold_duty[2] = 950U; /* module 6 */
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
    runtime_profiles[motor] = claw_module_profiles[(CLAW_PROFILE_GROUP - 1) * 3 + motor];
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    set_pwm(motor, 0U);
    motors[motor].position_counts = 0;
    motors[motor].hall_code = read_hall_code(motor);
    motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
    motors[motor].invalid_edges = 0U;
    motors[motor].wrong_direction_edges = 0U;
    motors[motor].last_hall_tick = now;
    motors[motor].start_counts = 0;
    motors[motor].command_direction = 0;
    motors[motor].duty_permille = 0U;
  }
  set_brake(true);
  set_direction(1, (uint8_t)((1U << CLAW_MOTOR_COUNT) - 1U));
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    motors[motor].command_direction = 0;
  }

  claw_state = CLAW_STATE_IDLE;
  fault_mask = 0U;
  last_command_sequence = 0U;
  last_command_result = (uint8_t)CLAW_RESULT_OK;
  last_command_tick = now;
  target_counts = 0;
  new_fault_pending = false;
  action_sequence_valid = false;
  move_start_tick = now;
  motion_enable_tick = now;
  trajectory_ms = 1U;
  last_control_tick = now;
  sync_fault_start_tick = 0U;
  diagnostic_counter = 0U;
  requested_speed_drpm = CLAW_DEFAULT_SPEED_DRPM;
  active_motor_mask = (uint8_t)((1U << CLAW_MOTOR_COUNT) - 1U);
  zero_calibrated = false;
  jog_active = false;
  run_in_active = false;
  seek_active = false;
  grip_active = false;
  seek_remaining_mask = 0U;
  jog_stop_tick = now;
}

static ClawResult start_jog(const ClawCommand *command)
{
  uint32_t now = HAL_GetTick();
  uint8_t motor;
  /* For JOG only: flags=raw DIR level, target=reserved zero, speed=motor 1..3. */
  if (command->flags > 1U || command->target_cdeg != 0 ||
      command->speed_drpm < 1U || command->speed_drpm > CLAW_MOTOR_COUNT)
    return CLAW_RESULT_RANGE;
  if (claw_state == CLAW_STATE_FAULT) return CLAW_RESULT_FAULT;
  if (claw_state == CLAW_STATE_MOVING) return CLAW_RESULT_BUSY;
  motor = command->speed_drpm - 1U;
  if (now - jog_stop_tick < CLAW_DIRECTION_QUIET_MS ||
      now - motors[motor].last_hall_tick < CLAW_DIRECTION_QUIET_MS)
    return CLAW_RESULT_BUSY;
  motors[motor].hall_code = read_hall_code(motor);
  motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
  if (motors[motor].previous_sector < 0)
  {
    latch_fault((uint8_t)(1U << motor));
    return CLAW_RESULT_FAULT;
  }
  stop_all(true);
  motors[motor].invalid_edges = 0U;
  motors[motor].wrong_direction_edges = 0U;
  motors[motor].start_counts = motors[motor].position_counts;
  /* Direction-to-Hall sign is precisely what this test measures. */
  HAL_GPIO_WritePin(dir_port[motor], dir_pin[motor],
                    command->flags ? GPIO_PIN_SET : GPIO_PIN_RESET);
  active_motor_mask = (uint8_t)(1U << motor);
  jog_motor = motor;
  jog_limit_counts = CLAW_JOG_MAX_COUNTS;
  if (CLAW_PROFILE_GROUP == 2 && motor == 2U && command->flags == 0U)
    jog_limit_counts = CLAW_JOG_M6_DIR0_MAX_COUNTS;
  jog_active = true;
  move_start_tick = now;
  motion_enable_tick = now + CLAW_DIRECTION_SETTLE_MS;
  claw_state = CLAW_STATE_MOVING;
  return CLAW_RESULT_OK;
}

static ClawResult start_run_in(const ClawCommand *command)
{
  uint32_t now = HAL_GetTick();
  uint8_t motor;
  uint8_t mask = command->speed_drpm == 0U ? 7U : command->speed_drpm;
  if (command->flags > 1U || command->target_cdeg != 0 ||
      command->speed_drpm > 7U || (mask & ~7U) != 0U)
    return CLAW_RESULT_RANGE;
  if ((CLAW_COMMISSIONED_MASK & mask) != mask) return CLAW_RESULT_INVALID;
  if (claw_state == CLAW_STATE_FAULT) return CLAW_RESULT_FAULT;
  if (claw_state == CLAW_STATE_MOVING) return CLAW_RESULT_BUSY;
  if (now - jog_stop_tick < CLAW_DIRECTION_QUIET_MS) return CLAW_RESULT_BUSY;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((mask & (uint8_t)(1U << motor)) == 0U) continue;
    motors[motor].hall_code = read_hall_code(motor);
    motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
    if (motors[motor].previous_sector < 0)
    {
      latch_fault((uint8_t)(1U << motor));
      return CLAW_RESULT_FAULT;
    }
    motors[motor].invalid_edges = 0U;
    motors[motor].wrong_direction_edges = 0U;
    motors[motor].last_hall_tick = now;
  }
  stop_all(true);
  active_motor_mask = mask;
  set_direction(command->flags ? 1 : -1, mask);
  motion_enable_tick = now + CLAW_DIRECTION_SETTLE_MS;
  run_in_active = true;
  claw_state = CLAW_STATE_MOVING;
  return CLAW_RESULT_OK;
}

/* A no-progress stop is a candidate endpoint only; never sets zero. */
static ClawResult start_seek_open(const ClawCommand *command)
{
  uint32_t now = HAL_GetTick();
  uint8_t motor;
  /* Byte6: 0=all three, 1..3=one local motor. Validate all before enabling any. */
  if (command->flags != 0 || command->target_cdeg != 0 || command->speed_drpm > 3)
    return CLAW_RESULT_RANGE;
  if (command->opcode == CLAW_COMMAND_GRIP && grip_active) return CLAW_RESULT_BUSY;
  if (command->opcode == CLAW_COMMAND_GRIP && command->speed_drpm != 0) return CLAW_RESULT_RANGE;
  uint8_t mask = command->speed_drpm == 0 ? 7U : (uint8_t)(1U << (command->speed_drpm - 1U));
  if ((CLAW_COMMISSIONED_MASK & mask) != mask) return CLAW_RESULT_INVALID;
  if (claw_state == CLAW_STATE_FAULT) return CLAW_RESULT_FAULT;
  if (claw_state == CLAW_STATE_MOVING) return CLAW_RESULT_BUSY;
  if (now - jog_stop_tick < CLAW_DIRECTION_QUIET_MS) return CLAW_RESULT_BUSY;
  for (motor=0; motor<CLAW_MOTOR_COUNT; ++motor)
    if ((mask & (1U << motor)) && now - motors[motor].last_hall_tick < CLAW_DIRECTION_QUIET_MS)
      return CLAW_RESULT_BUSY;
  for (motor=0; motor<CLAW_MOTOR_COUNT; ++motor)
  {
    if (!(mask & (1U << motor))) continue;
    motors[motor].hall_code = read_hall_code(motor);
    motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
    if (motors[motor].previous_sector < 0)
    {
      latch_fault((uint8_t)(1U << motor));
      return CLAW_RESULT_FAULT;
    }
  }
  stop_all(true);
  active_motor_mask = mask;
  move_start_tick = now;
  motion_enable_tick = now + CLAW_DIRECTION_SETTLE_MS;
  for (motor=0; motor<CLAW_MOTOR_COUNT; ++motor)
  {
    if (!(mask & (1U << motor))) continue;
    motors[motor].invalid_edges = 0U;
    motors[motor].wrong_direction_edges = 0U;
    motors[motor].start_counts = motors[motor].position_counts;
    motors[motor].last_hall_tick = motion_enable_tick;
  }
  grip_active = command->opcode == CLAW_COMMAND_GRIP;
  hold_cld_reset_tick = now;
  for (motor=0; motor<CLAW_MOTOR_COUNT; ++motor) startup_done[motor] = false;
  set_direction(grip_active ? 1 : -1, mask);
  seek_remaining_mask = mask;
  seek_active = true;
  claw_state = CLAW_STATE_MOVING;
  return CLAW_RESULT_OK;
}

static ClawResult start_move(int32_t requested_target_counts,
                             uint8_t speed_drpm,
                             uint8_t motor_mask)
{
  uint8_t motor;
  uint8_t invalid_hall_mask = 0U;

  uint32_t now;


  if (claw_state == CLAW_STATE_FAULT)
  {
    return CLAW_RESULT_FAULT;
  }
  if (claw_state == CLAW_STATE_MOVING)
  {
    return CLAW_RESULT_BUSY;
  }
  /* Direction, Hall sign and startup input must be confirmed on THIS board. */
  if ((motor_mask & CLAW_COMMISSIONED_MASK) != motor_mask)
    return CLAW_RESULT_INVALID;
  if (CLAW_TRIAL_MAX_STEP_MDEG > 0L)
  {
    if (CLAW_PROFILE_GROUP != 2) return CLAW_RESULT_INVALID;
    for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
      if ((motor_mask & (1U << motor)) &&
          abs_i32(requested_target_counts - motors[motor].position_counts) >
          ClawControl_AngleMilliDegToCounts(CLAW_TRIAL_MAX_STEP_MDEG))
        return CLAW_RESULT_RANGE;
  }
  if (active_motor_count(motor_mask) == 0U)
  {
    return CLAW_RESULT_INVALID;
  }

  now = HAL_GetTick();
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
    if ((motor_mask & (1U << motor)) &&
        now - motors[motor].last_hall_tick < CLAW_DIRECTION_QUIET_MS)
      return CLAW_RESULT_BUSY;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((motor_mask & (uint8_t)(1U << motor)) == 0U)
    {
      continue;
    }
    motors[motor].hall_code = read_hall_code(motor);
    motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
    if (motors[motor].previous_sector < 0)
    {
      invalid_hall_mask |= (uint8_t)(1U << motor);
    }
  }
  if (invalid_hall_mask != 0U)
  {
    latch_fault(invalid_hall_mask);
    return CLAW_RESULT_FAULT;
  }

  /* Reject invalid edited profiles before energizing any output. */
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    const ClawPidConfig *p = profile(motor);
    if (!(motor_mask & (1U << motor))) continue;
    if (!(p->kp >= 0 && p->kp <= 1000 && p->ki >= 0 && p->ki <= 1000 &&
          p->kd >= 0 && p->kd <= 1000 && p->integral_limit >= 0 &&
          p->integral_limit <= 700) || p->minimum > p->maximum ||
        p->maximum > CLAW_PWM_SAFE_MAX_PERMILLE || p->maximum == 0 ||
        p->startup > p->maximum || p->startup == 0 ||
        p->startup_counts == 0 || p->startup_ms < CLAW_CONTROL_PERIOD_MS ||
        p->startup_ms > CLAW_STALL_TIMEOUT_MS)
      return CLAW_RESULT_RANGE;
  }
  stop_all(true); /* Release any prior PWM grip before starting a position action. */
  requested_speed_drpm = speed_drpm ? speed_drpm : CLAW_DEFAULT_SPEED_DRPM;
  longest_path = 0;
  target_counts = requested_target_counts;
  active_motor_mask = motor_mask;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    motors[motor].start_counts = motors[motor].position_counts;
    motors[motor].wrong_direction_edges = 0U;
    motors[motor].invalid_edges = 0U;
    motors[motor].last_hall_tick = now;
    set_pwm(motor, 0U);
    ClawPid_Reset(&pid_state[motor], 0.0f);
    startup_done[motor] = false;
    path_counts[motor] = abs_i32(target_counts - motors[motor].start_counts);
    motor_done[motor] = path_counts[motor] <= CLAW_POSITION_DEADBAND_COUNTS;
    motors[motor].command_direction = 0;
    if ((motor_mask & (1U << motor)) != 0U)
    {
      set_direction(target_counts >= motors[motor].start_counts ? 1 : -1,
                    (uint8_t)(1U << motor));
      if (path_counts[motor] > longest_path) longest_path = path_counts[motor];
    }
  }
  set_brake(true);
  /* speed is a nominal trajectory rate, never a measured shaft speed. */
  trajectory_ms = (uint32_t)(((int64_t)ClawControl_CountsToAngleMilliDeg(longest_path)
                            * 10 + 6 * requested_speed_drpm - 1)
                           / (6 * requested_speed_drpm));
  if (trajectory_ms == 0U) trajectory_ms = 1U;
  move_start_tick = now;
  motion_enable_tick = now + CLAW_DIRECTION_SETTLE_MS;

  sync_fault_start_tick = 0U;
  claw_state = CLAW_STATE_MOVING;
  return CLAW_RESULT_OK;
}

/* IDs 1..9/14: per-motor; 10..13/15..16: global. Gains use tenths. */
static bool parameter_read(uint8_t channel, uint8_t id, int16_t *value)
{
  if (channel == 0) {
    switch (id) {
      case 10: *value=seek_duty; return true;
      case 11: *value=seek_no_progress_ms; return true;
      case 12: *value=seek_timeout_ms; return true;
      case 13: *value=CLAW_PROFILE_GROUP; return true;
      case 15: *value=grip_no_progress_ms; return true;
      case 16: *value=grip_timeout_ms; return true;
      default: return false;
    }
  }
  if (channel > 3) return false;
  if (id == 18) { *value=hold_duty[channel-1]; return true; }
  if (id == 17) { *value=seek_motor_duty[channel-1]; return true; }
  if (id == 14) { *value=grip_duty[channel-1]; return true; }
  const ClawPidConfig *p=profile(channel-1);
  switch(id) {
    case 1: *value=(int16_t)(p->kp*10+0.5f); break;
    case 2: *value=(int16_t)(p->ki*10+0.5f); break;
    case 3: *value=(int16_t)(p->kd*10+0.5f); break;
    case 4: *value=(int16_t)p->integral_limit; break;
    case 5: *value=p->minimum; break;
    case 6: *value=p->maximum; break;
    case 7: *value=p->startup; break;
    case 8: *value=p->startup_counts; break;
    case 9: *value=p->startup_ms; break;
    default: return false;
  }
  return true;
}

static ClawResult parameter_command(const ClawCommand *command)
{
  int16_t old_value;
  uint8_t ch=command->speed_drpm, id=command->flags;
  int16_t value=command->target_cdeg;
  if (!parameter_read(ch,id,&old_value)) return CLAW_RESULT_INVALID;
  if (command->opcode == CLAW_COMMAND_PARAM_GET)
    return value == 0 ? CLAW_RESULT_OK : CLAW_RESULT_RANGE;
  if (claw_state == CLAW_STATE_MOVING || grip_active) return CLAW_RESULT_BUSY;
  if (value < 0) return CLAW_RESULT_RANGE;
  if (ch == 0) {
    if (id==10 && value>=0 && value<=1000) {
      seek_duty=(uint16_t)value;
      for (unsigned m=0;m<3;++m) seek_motor_duty[m]=seek_duty;
    }
    else if (id==11 && value>=100 && value<=2000) seek_no_progress_ms=(uint16_t)value;
    else if (id==12 && value>=2500 && value<=15000) seek_timeout_ms=(uint16_t)value;
    else if (id==15 && value>=100 && value<=2000) grip_no_progress_ms=(uint16_t)value;
    else if (id==16 && value>=2500 && value<=15000) grip_timeout_ms=(uint16_t)value;
    else return CLAW_RESULT_RANGE;
    return CLAW_RESULT_OK;
  }
  if (id == 18) {
    if(value>1000) return CLAW_RESULT_RANGE;
    hold_duty[ch-1]=(uint16_t)value;
    return CLAW_RESULT_OK;
  }
  if (id == 17) {
    if(value<0 || value>1000) return CLAW_RESULT_RANGE;
    seek_motor_duty[ch-1]=(uint16_t)value;
    return CLAW_RESULT_OK;
  }
  if (id == 14) {
    if(value<0 || value>1000) return CLAW_RESULT_RANGE;
    grip_duty[ch-1]=(uint16_t)value;
    return CLAW_RESULT_OK;
  }
  ClawPidConfig next=*profile(ch-1);
  switch(id) {
    case 1: if(value>10000) return CLAW_RESULT_RANGE; next.kp=value/10.0f; break;
    case 2: if(value>10000) return CLAW_RESULT_RANGE; next.ki=value/10.0f; break;
    case 3: if(value>10000) return CLAW_RESULT_RANGE; next.kd=value/10.0f; break;
    case 4: if(value>700) return CLAW_RESULT_RANGE; next.integral_limit=value; break;
    case 5: next.minimum=(uint16_t)value; break;
    case 6: next.maximum=(uint16_t)value; break;
    case 7: next.startup=(uint16_t)value; break;
    case 8: if(value<1 || value>100) return CLAW_RESULT_RANGE; next.startup_counts=(uint16_t)value; break;
    case 9: if(value<10 || (uint16_t)value>CLAW_STALL_TIMEOUT_MS) return CLAW_RESULT_RANGE; next.startup_ms=(uint16_t)value; break;
    default: return CLAW_RESULT_INVALID;
  }
  if (next.maximum<1 || next.maximum>CLAW_PWM_SAFE_MAX_PERMILLE ||
      next.minimum>next.maximum || next.startup<1 || next.startup>next.maximum)
    return CLAW_RESULT_RANGE;
  runtime_profiles[ch-1]=next;
  ClawPid_Reset(&pid_state[ch-1],0);
  return CLAW_RESULT_OK;
}

void ClawControl_BuildParameterFrame(const ClawCommand *command, uint8_t data[8])
{
  int16_t value=0;
  (void)parameter_read(command->speed_drpm,command->flags,&value);
  data[0]=CLAW_PROTOCOL_VERSION; data[1]=command->sequence;
  data[2]=command->flags; data[3]=command->speed_drpm;
  data[4]=(uint8_t)value; data[5]=(uint8_t)((uint16_t)value>>8);
  data[6]=last_command_result; data[7]=ClawProtocol_Crc8(data,7);
}

ClawResult ClawControl_HandleCommand(const ClawCommand *command)
{
  int32_t target_mdeg;
  uint8_t motor;
  uint8_t motor_mask;
  uint8_t selected_count;

  if (command == NULL)
  {
    return record_command_result(CLAW_RESULT_INVALID);
  }

  last_command_tick = HAL_GetTick();
  last_command_sequence = command->sequence;
  if (command->opcode == CLAW_COMMAND_HEARTBEAT)
  {
    return CLAW_RESULT_OK;
  }

  if (action_sequence_valid && (command->sequence == last_action_sequence))
  {
    return (ClawResult)last_command_result;
  }
  action_sequence_valid = true;
  last_action_sequence = command->sequence;

  switch (command->opcode)
  {
    case CLAW_COMMAND_PARAM_SET:
    case CLAW_COMMAND_PARAM_GET:
      return record_command_result(parameter_command(command));

    case CLAW_COMMAND_GRIP:
    case CLAW_COMMAND_SEEK_OPEN:
      return record_command_result(start_seek_open(command));

    case CLAW_COMMAND_JOG:
      return record_command_result(start_jog(command));

    case CLAW_COMMAND_RUN_IN:
      return record_command_result(start_run_in(command));

    case CLAW_COMMAND_SET_CAN_ID:
      if (claw_state == CLAW_STATE_MOVING || grip_active) return record_command_result(CLAW_RESULT_BUSY);
      return record_command_result(Board_SetCanBaseId((uint16_t)command->target_cdeg,
                                                       (command->flags & 1U) != 0U) ?
                                   CLAW_RESULT_OK : CLAW_RESULT_RANGE);

    case CLAW_COMMAND_SET_TARGET:
    case CLAW_COMMAND_SET_TARGET_M1:
    case CLAW_COMMAND_SET_TARGET_M2:
    case CLAW_COMMAND_SET_TARGET_M3:
      motor_mask = command_motor_mask(command->opcode);
      selected_count = active_motor_count(motor_mask);
      if ((command->speed_drpm != 0U) &&
          ((command->speed_drpm < CLAW_MIN_SPEED_DRPM) ||
           (command->speed_drpm > CLAW_MAX_SPEED_DRPM)))
      {
        return record_command_result(CLAW_RESULT_RANGE);
      }
      target_mdeg = (int32_t)command->target_cdeg * 10L;
      if ((command->flags & CLAW_COMMAND_FLAG_RELATIVE) != 0U)
      {
        int32_t position_sum = 0;
        for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
        {
          if ((motor_mask & (uint8_t)(1U << motor)) != 0U)
          {
            position_sum += motors[motor].position_counts;
          }
        }
        target_mdeg += ClawControl_CountsToAngleMilliDeg(
            position_sum / (int32_t)selected_count);
        /* After zeroing, relative open commands stop at the physical-zero
           coordinate. Before zeroing, allow a bounded negative commissioning
           range so the user can reach the mechanical open limit first. */
        if (target_mdeg < (zero_calibrated ?
                           CLAW_MIN_TARGET_MDEG :
                           CLAW_PREZERO_MIN_TARGET_MDEG))
        {
          target_mdeg = zero_calibrated ?
                        CLAW_MIN_TARGET_MDEG :
                        CLAW_PREZERO_MIN_TARGET_MDEG;
        }
        else if (target_mdeg > CLAW_MAX_TARGET_MDEG)
        {
          target_mdeg = CLAW_MAX_TARGET_MDEG;
        }
      }
      else if ((target_mdeg < CLAW_MIN_TARGET_MDEG) ||
               (target_mdeg > CLAW_MAX_TARGET_MDEG))
      {
        return record_command_result(CLAW_RESULT_RANGE);
      }

      /* A pure synchronization fault stops the group, but it should still
       * permit a technician to recover one selected motor with f1/f2/f3.
       * Do not bypass motor, Hall, communication, or mixed faults.  The
       * selected motor is revalidated by start_move() below. */
      if (claw_state == CLAW_STATE_FAULT)
      {
        if ((fault_mask != CLAW_FAULT_SYNC) || (selected_count != 1U))
        {
          return record_command_result(CLAW_RESULT_FAULT);
        }
        fault_mask = 0U;
        new_fault_pending = false;
        sync_fault_start_tick = 0U;
        claw_state = CLAW_STATE_IDLE;
      }
      return record_command_result(
          start_move(ClawControl_AngleMilliDegToCounts(target_mdeg),
                     command->speed_drpm, motor_mask));

    case CLAW_COMMAND_STOP:
      stop_all(true);
      claw_state = (fault_mask == 0U) ? CLAW_STATE_IDLE : CLAW_STATE_FAULT;
      return record_command_result(CLAW_RESULT_OK);

    case CLAW_COMMAND_CLEAR_FAULT:
      stop_all(true);
      for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
      {
        motors[motor].invalid_edges = 0U;
        motors[motor].wrong_direction_edges = 0U;
        motors[motor].hall_code = read_hall_code(motor);
        motors[motor].previous_sector = hall_to_sector(motors[motor].hall_code);
        if (motors[motor].previous_sector < 0)
        {
          return record_command_result(CLAW_RESULT_FAULT);
        }
      }
      if (!Board_ReleaseOutputs()) return record_command_result(CLAW_RESULT_FAULT);
      fault_mask = 0U;
      new_fault_pending = false;
      sync_fault_start_tick = 0U;
      claw_state = CLAW_STATE_IDLE;
      return record_command_result(CLAW_RESULT_OK);

    case CLAW_COMMAND_SET_ZERO:
      if (claw_state == CLAW_STATE_MOVING || grip_active)
      {
        return record_command_result(CLAW_RESULT_BUSY);
      }
      for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
      {
        motors[motor].position_counts = 0;
        motors[motor].start_counts = 0;
      }
      target_counts = 0;
      zero_calibrated = true;
      claw_state = (fault_mask == 0U) ? CLAW_STATE_IDLE : CLAW_STATE_FAULT;
      return record_command_result(CLAW_RESULT_OK);

    default:
      return record_command_result(CLAW_RESULT_INVALID);
  }
}

void ClawControl_HallEdgeIRQ(uint16_t gpio_pin_value)
{
  uint8_t motor;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    uint8_t phase;
    bool pin_matches = false;
    for (phase = 0U; phase < 3U; ++phase)
    {
      if (hall_pin[motor][phase] == gpio_pin_value)
      {
        pin_matches = true;
      }
    }

    if (pin_matches)
    {
      uint8_t code = read_hall_code(motor);
      int8_t sector = hall_to_sector(code);
      int8_t previous = motors[motor].previous_sector;
      motors[motor].hall_code = code;

      if (sector < 0)
      {
        if (++motors[motor].invalid_edges >= 3U && (claw_state == CLAW_STATE_MOVING || grip_active))
        {
          latch_fault((uint8_t)(1U << motor));
        }
        return;
      }

      if (previous >= 0 && sector != previous)
      {
        int8_t difference = (int8_t)(sector - previous);
        int8_t raw_step = 0;
        int8_t logical_step;
        if ((difference == 1) || (difference == -5))
        {
          raw_step = 1;
        }
        else if ((difference == -1) || (difference == 5))
        {
          raw_step = -1;
        }

        if (raw_step == 0)
        {
          if (++motors[motor].invalid_edges >= 3U && (claw_state == CLAW_STATE_MOVING || grip_active))
          {
            latch_fault((uint8_t)(1U << motor));
          }
        }
        else
        {
          logical_step = (int8_t)(raw_step * hall_count_sign[motor]);
          motors[motor].position_counts += logical_step;
          motors[motor].last_hall_tick = HAL_GetTick();
          motors[motor].invalid_edges = 0U;
          if ((claw_state == CLAW_STATE_MOVING || grip_active) &&
              ((int32_t)(HAL_GetTick() - motion_enable_tick) >= 0) &&
              (motors[motor].command_direction != 0) &&
              (logical_step != motors[motor].command_direction))
          {
            if (++motors[motor].wrong_direction_edges >= 3U)
            {
              latch_fault((uint8_t)(1U << motor));
            }
          }
          else
          {
            motors[motor].wrong_direction_edges = 0U;
          }
        }
      }
      motors[motor].previous_sector = sector;
      return;
    }
  }
}

void ClawControl_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t motor;
  uint8_t selected_count;
  float dt = (float)(now - last_control_tick) / 1000.0f;
  float fraction;
  int32_t min_position = 0;
  int32_t max_position = 0;
  bool have_position = false;
  bool all_done = true;


  if ((claw_state == CLAW_STATE_MOVING || grip_active) &&
      ((now - last_command_tick) > CLAW_COMMAND_WATCHDOG_MS))
  {
    latch_fault(CLAW_FAULT_COMM_TIMEOUT);
    return;
  }
  if ((now - last_control_tick) < CLAW_CONTROL_PERIOD_MS)
  {
    return;
  }
  last_control_tick = now;
  if (claw_state == CLAW_STATE_HOLD && grip_active)
  {
    uint32_t since_reset = now - hold_cld_reset_tick;
    if (since_reset >= CLAW_HOLD_CLD_RESET_PERIOD_MS)
      hold_cld_reset_tick = now;
    since_reset = now - hold_cld_reset_tick;
    for (motor=0U; motor<CLAW_MOTOR_COUNT; ++motor)
      set_pwm(motor, since_reset < CLAW_HOLD_CLD_RESET_PULSE_MS ? 0U : hold_duty[motor]);
    return;
  }
  if (claw_state != CLAW_STATE_MOVING)
  {
    return;
  }
  if (seek_active)
  {
    if ((int32_t)(now - motion_enable_tick) < 0) return;
    if (now - motion_enable_tick >= (grip_active ? grip_timeout_ms : seek_timeout_ms))
    {
      latch_fault(CLAW_FAULT_SYNC); /* deadline is NOT an endpoint */
      return;
    }
    set_brake(false);
    for (motor=0; motor<CLAW_MOTOR_COUNT; ++motor)
    {
      uint8_t bit = (uint8_t)(1U << motor);
      if (!(seek_remaining_mask & bit)) { set_pwm(motor, grip_active ? hold_duty[motor] : 0U); continue; }
      if (now - motors[motor].last_hall_tick >= (grip_active ? grip_no_progress_ms : seek_no_progress_ms))
      {
        set_pwm(motor, grip_active ? hold_duty[motor] : 0U);
        if (!grip_active) motors[motor].command_direction = 0;
        seek_remaining_mask &= (uint8_t)~bit; /* latched: coast cannot restart it */
      }
      else if (grip_active) {
        const ClawPidConfig *p = profile(motor);
        /* Bounded startup assist; thereafter constant per-motor PWM, no PID. */
        if (motors[motor].position_counts - motors[motor].start_counts >= p->startup_counts ||
            now - motion_enable_tick >= p->startup_ms) startup_done[motor] = true;
        set_pwm(motor, startup_done[motor] ? grip_duty[motor] : p->startup);
      }
      else set_pwm(motor, seek_motor_duty[motor]);
    }
    if (seek_remaining_mask == 0U)
    {
      if (grip_active) {
        seek_active = false;
        hold_cld_reset_tick = now;
        claw_state = CLAW_STATE_HOLD; /* Open-loop PWM hold; not verified grasp. */
      } else {
        stop_all(true);
        claw_state = CLAW_STATE_IDLE; /* physical endpoints still need confirmation */
      }
    }
    return;
  }
  if (jog_active)
  {
    /* Absolute deadline: repeated commands/heartbeats cannot extend the pulse. */
    if (now - move_start_tick >= CLAW_DIRECTION_SETTLE_MS + CLAW_JOG_DURATION_MS ||
        abs_i32(motors[jog_motor].position_counts - motors[jog_motor].start_counts)
          >= jog_limit_counts)
    {
      stop_all(true);
      claw_state = CLAW_STATE_IDLE;
    }
    else if ((int32_t)(now - motion_enable_tick) >= 0)
    {
      set_brake(false);
      set_pwm(jog_motor, CLAW_JOG_DUTY_PERMILLE);
    }
    return;
  }
  if (run_in_active)
  {
    if ((int32_t)(now - motion_enable_tick) >= 0)
    {
      set_brake(false);
      for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
        set_pwm(motor, (active_motor_mask & (uint8_t)(1U << motor)) ?
                       CLAW_RUN_IN_DUTY_PERMILLE : 0U);
    }
    return;
  }
  if ((now - move_start_tick) > CLAW_MOVE_TIMEOUT_MS)
  {
    latch_fault(CLAW_FAULT_SYNC);
    return;
  }
  if ((int32_t)(now - motion_enable_tick) < 0)
  {
    return;
  }

  selected_count = active_motor_count(active_motor_mask);
  if (selected_count == 0U)
  {
    latch_fault(CLAW_FAULT_SYNC);
    return;
  }

  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((active_motor_mask & (uint8_t)(1U << motor)) == 0U)
    {
      set_pwm(motor, 0U);
      continue;
    }
    int32_t error = target_counts - motors[motor].position_counts;
    int32_t progress = (target_counts >= motors[motor].start_counts ? 1 : -1)
                     * (motors[motor].position_counts - motors[motor].start_counts);
    /* Compare normalized progress in longest-path counts. This tolerates
       unequal starts and opposite per-finger directions. */
    int32_t sync_position = path_counts[motor] > CLAW_POSITION_DEADBAND_COUNTS ?
        (int32_t)((int64_t)progress * longest_path / path_counts[motor]) : longest_path;
    if (sync_position > longest_path) sync_position = longest_path;
    if (path_counts[motor] <= CLAW_POSITION_DEADBAND_COUNTS) continue;
    if (!have_position)
    {
      min_position = sync_position;
      max_position = sync_position;
      have_position = true;
    }
    else if (sync_position < min_position)
    {
      min_position = sync_position;
    }
    if (sync_position > max_position)
    {
      max_position = sync_position;
    }
    if (((target_counts >= motors[motor].start_counts ? 1 : -1) * error)
        <= CLAW_POSITION_DEADBAND_COUNTS) motor_done[motor] = true;
    if (!motor_done[motor]) all_done = false;
  }

  if (all_done)
  {
    stop_all(true);
    claw_state = CLAW_STATE_HOLD;
    return;
  }

  if ((selected_count > 1U) &&
      ((max_position - min_position) > CLAW_SYNC_FAULT_COUNTS))
  {
    if (sync_fault_start_tick == 0U)
    {
      sync_fault_start_tick = now;
    }
    else if ((now - sync_fault_start_tick) > CLAW_SYNC_FAULT_TIME_MS)
    {
      latch_fault(CLAW_FAULT_SYNC);
      return;
    }
  }
  else
  {
    sync_fault_start_tick = 0U;
  }

  /* BRAKE is shared; a finger that arrives early is stopped with PWM = 0. */
  set_brake(false);
  fraction = (float)(now - motion_enable_tick) / (float)trajectory_ms;
  if (fraction > 1.0f) fraction = 1.0f;

  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    if ((active_motor_mask & (uint8_t)(1U << motor)) == 0U)
    {
      set_pwm(motor, 0U);
      continue;
    }
    int32_t progress = (target_counts >= motors[motor].start_counts ? 1 : -1)
                     * (motors[motor].position_counts - motors[motor].start_counts);
    const ClawPidConfig *p = profile(motor);
    uint16_t duty;
    uint16_t ceiling = p->maximum;

    if (motor_done[motor])
    {
      set_pwm(motor, 0U);
      motors[motor].command_direction = 0;
      ClawPid_Reset(&pid_state[motor], (float)progress);
      continue;
    }
    if (!startup_done[motor])
    {
      if (progress >= p->startup_counts)
      {
        startup_done[motor] = true;
        ClawPid_Reset(&pid_state[motor], (float)progress);
      }
      else if ((now - motion_enable_tick) >= p->startup_ms)
      {
        latch_fault((uint8_t)(1U << motor));
        return;
      }
      else
      {
        set_pwm(motor, p->startup);
        continue;
      }
    }
    /* A deliberate zero-PWM wait must not count as a powered stall. */
    if (motors[motor].duty_permille == 0U) motors[motor].last_hall_tick = now;
    if ((now - motors[motor].last_hall_tick) > CLAW_STALL_TIMEOUT_MS)
    {
      latch_fault((uint8_t)(1U << motor));
      return;
    }
    duty = ClawPid_Step(&pid_state[motor], p, fraction * path_counts[motor],
                        (float)progress, dt, ceiling);
    set_pwm(motor, duty);
  }
}

void ClawControl_GetStatus(ClawStatus *status)
{
  uint32_t key;
  uint8_t motor;
  uint8_t status_motor_mask;
  int32_t min_position = 0;
  int32_t max_position = 0;
  bool have_position = false;
  if (status == NULL)
  {
    return;
  }

  key = irq_lock();
  status->state = claw_state;
  status->fault_mask = fault_mask;
  status->last_command_sequence = last_command_sequence;
  status->last_command_result = last_command_result;
  status->target_counts = target_counts;
  status_motor_mask = (claw_state == CLAW_STATE_MOVING) ?
                      active_motor_mask :
                      (uint8_t)((1U << CLAW_MOTOR_COUNT) - 1U);
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    status->position_counts[motor] = motors[motor].position_counts;
    status->hall_code[motor] = motors[motor].hall_code;
    status->duty_permille[motor] = motors[motor].duty_permille;
    if ((status_motor_mask & (uint8_t)(1U << motor)) == 0U)
    {
      continue;
    }
    if (!have_position)
    {
      min_position = motors[motor].position_counts;
      max_position = motors[motor].position_counts;
      have_position = true;
    }
    else if (motors[motor].position_counts < min_position)
    {
      min_position = motors[motor].position_counts;
    }
    if (motors[motor].position_counts > max_position)
    {
      max_position = motors[motor].position_counts;
    }
  }
  status->last_command_age_ms = HAL_GetTick() - last_command_tick;
  irq_unlock(key);
  status->synchronization_error_cdeg = clamp_i16(
      have_position ?
      (ClawControl_CountsToAngleMilliDeg(max_position - min_position) / 10L) : 0L);
}

static void write_i16_le(uint8_t *data, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  data[0] = (uint8_t)raw;
  data[1] = (uint8_t)(raw >> 8);
}

void ClawControl_BuildPositionFrame(uint8_t data[8])
{
  ClawStatus status;
  uint8_t motor;
  ClawControl_GetStatus(&status);
  data[0] = CLAW_PROTOCOL_VERSION;
  data[1] = (uint8_t)status.state;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    int16_t angle_cdeg = clamp_i16(
        ClawControl_CountsToAngleMilliDeg(status.position_counts[motor]) / 10L);
    write_i16_le(&data[2U + (2U * motor)], angle_cdeg);
  }
}

void ClawControl_BuildStatusFrame(uint8_t data[8])
{
  ClawStatus status;
  uint32_t age_10ms;
  ClawControl_GetStatus(&status);
  age_10ms = status.last_command_age_ms / 10U;
  data[0] = CLAW_PROTOCOL_VERSION;
  data[1] = status.last_command_sequence;
  data[2] = (uint8_t)status.state;
  data[3] = status.fault_mask;
  write_i16_le(&data[4], status.synchronization_error_cdeg);
  data[6] = (age_10ms > 255U) ? 255U : (uint8_t)age_10ms;
  data[7] = (uint8_t)(status.last_command_result & 0x0FU);
  if (status.state == CLAW_STATE_HOLD && !grip_active)
  {
    data[7] |= 0x80U;
  }
}

void ClawControl_BuildDiagnosticFrame(uint8_t data[8])
{
  ClawStatus status;
  uint8_t motor;
  ClawControl_GetStatus(&status);
  data[0] = CLAW_PROTOCOL_VERSION;
  for (motor = 0U; motor < CLAW_MOTOR_COUNT; ++motor)
  {
    data[1U + motor] = status.hall_code[motor];
    data[4U + motor] = (uint8_t)(status.duty_permille[motor] / 4U);
  }
  data[7] = diagnostic_counter++;
}

bool ClawControl_ConsumeNewFault(void)
{
  bool pending;
  uint32_t key = irq_lock();
  pending = new_fault_pending;
  new_fault_pending = false;
  irq_unlock(key);
  return pending;
}

void ClawControl_TransportFault(void)
{
  latch_fault(CLAW_FAULT_COMM_TIMEOUT);
}
