#ifndef CLAW_CONTROL_H
#define CLAW_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "claw_protocol.h"

typedef enum
{
  CLAW_STATE_IDLE = 0,
  CLAW_STATE_MOVING = 1,
  CLAW_STATE_HOLD = 2,
  CLAW_STATE_FAULT = 3
} ClawState;

typedef enum
{
  CLAW_RESULT_OK = 0,
  CLAW_RESULT_BUSY,
  CLAW_RESULT_FAULT,
  CLAW_RESULT_RANGE,
  CLAW_RESULT_INVALID
} ClawResult;

#define CLAW_FAULT_MOTOR_1       0x01U
#define CLAW_FAULT_MOTOR_2       0x02U
#define CLAW_FAULT_MOTOR_3       0x04U
#define CLAW_FAULT_SYNC          0x40U
#define CLAW_FAULT_COMM_TIMEOUT  0x80U

typedef struct
{
  ClawState state;
  uint8_t fault_mask;
  uint8_t last_command_sequence;
  uint8_t last_command_result;
  int32_t position_counts[3];
  int32_t target_counts;
  uint8_t hall_code[3];
  uint16_t duty_permille[3];
  int16_t synchronization_error_cdeg;
  uint32_t last_command_age_ms;
} ClawStatus;

void ClawControl_Init(void);
void ClawControl_TransportFault(void);
void ClawControl_Task(void);
ClawResult ClawControl_HandleCommand(const ClawCommand *command);
void ClawControl_HallEdgeIRQ(uint16_t gpio_pin);
void ClawControl_GetStatus(ClawStatus *status);

void ClawControl_BuildParameterFrame(const ClawCommand *command, uint8_t data[8]);
void ClawControl_BuildPositionFrame(uint8_t data[8]);
void ClawControl_BuildStatusFrame(uint8_t data[8]);
void ClawControl_BuildDiagnosticFrame(uint8_t data[8]);
bool ClawControl_ConsumeNewFault(void);

int32_t ClawControl_AngleMilliDegToCounts(int32_t angle_mdeg);
int32_t ClawControl_CountsToAngleMilliDeg(int32_t counts);

#endif /* CLAW_CONTROL_H */
