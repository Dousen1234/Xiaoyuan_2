#ifndef CLAW_PROTOCOL_H
#define CLAW_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define CLAW_PROTOCOL_VERSION       1U
#define CLAW_CAN_DLC                8U

/* 11-bit CAN 2.0A identifiers. TiHu joint nodes remain in 0x001..0x07F. */
#define CLAW_CAN_ID_DEFAULT_BASE    11U
#define CLAW_CAN_ID_MIN_BASE        11U
#define CLAW_CAN_ID_MAX_BASE        120U
#define CLAW_CAN_ID_FAULT           (claw_can_id_base + 5U)
#define CLAW_CAN_ID_COMMAND         (claw_can_id_base + 0U)
#define CLAW_CAN_ID_POSITION        (claw_can_id_base + 1U)
#define CLAW_CAN_ID_STATUS          (claw_can_id_base + 2U)
#define CLAW_CAN_ID_DIAGNOSTIC      (claw_can_id_base + 3U)
#define CLAW_CAN_ID_PARAMETER       (claw_can_id_base + 4U)
extern volatile uint16_t claw_can_id_base;

#define CLAW_COMMAND_FLAG_RELATIVE  0x01U

typedef enum
{
  CLAW_COMMAND_SET_TARGET = 0x01,
  CLAW_COMMAND_STOP = 0x02,
  CLAW_COMMAND_CLEAR_FAULT = 0x03,
  CLAW_COMMAND_SET_ZERO = 0x04,
  CLAW_COMMAND_HEARTBEAT = 0x05,
  CLAW_COMMAND_SET_TARGET_M1 = 0x06,
  CLAW_COMMAND_SET_TARGET_M2 = 0x07,
  CLAW_COMMAND_SET_TARGET_M3 = 0x08,
  CLAW_COMMAND_JOG = 0x09,
  CLAW_COMMAND_SEEK_OPEN = 0x0A,
  CLAW_COMMAND_PARAM_SET = 0x0B,
  CLAW_COMMAND_PARAM_GET = 0x0C,
  CLAW_COMMAND_GRIP = 0x0D,
  CLAW_COMMAND_RUN_IN = 0x0E,
  CLAW_COMMAND_SET_CAN_ID = 0x0F
} ClawCommandOpcode;

typedef struct
{
  uint8_t sequence;
  ClawCommandOpcode opcode;
  uint8_t flags;
  int16_t target_cdeg;
  uint8_t speed_drpm;
} ClawCommand;

uint8_t ClawProtocol_Crc8(const uint8_t *data, uint8_t length);
bool ClawProtocol_DecodeCommand(const uint8_t data[CLAW_CAN_DLC], ClawCommand *command);
void ClawProtocol_EncodeCommand(const ClawCommand *command, uint8_t data[CLAW_CAN_DLC]);

#endif /* CLAW_PROTOCOL_H */
