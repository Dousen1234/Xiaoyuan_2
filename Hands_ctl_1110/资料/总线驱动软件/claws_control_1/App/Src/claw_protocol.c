#include "claw_protocol.h"

#include <stddef.h>

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void write_i16_le(uint8_t *data, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  data[0] = (uint8_t)raw;
  data[1] = (uint8_t)(raw >> 8);
}

uint8_t ClawProtocol_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFFU;
  uint8_t byte_index;

  if (data == NULL)
  {
    return 0U;
  }

  for (byte_index = 0U; byte_index < length; ++byte_index)
  {
    uint8_t bit_index;
    crc ^= data[byte_index];
    for (bit_index = 0U; bit_index < 8U; ++bit_index)
    {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1) ^ 0x1DU)
                                  : (uint8_t)(crc << 1);
    }
  }
  return (uint8_t)(crc ^ 0xFFU);
}

bool ClawProtocol_DecodeCommand(const uint8_t data[CLAW_CAN_DLC], ClawCommand *command)
{
  if ((data == NULL) || (command == NULL) ||
      (data[0] != CLAW_PROTOCOL_VERSION) ||
      (ClawProtocol_Crc8(data, 7U) != data[7]))
  {
    return false;
  }

  if ((data[2] < (uint8_t)CLAW_COMMAND_SET_TARGET) ||
      (data[2] > (uint8_t)CLAW_COMMAND_SET_CAN_ID))
  {
    return false;
  }

  command->sequence = data[1];
  command->opcode = (ClawCommandOpcode)data[2];
  command->flags = data[3];
  command->target_cdeg = read_i16_le(&data[4]);
  command->speed_drpm = data[6];
  return true;
}

void ClawProtocol_EncodeCommand(const ClawCommand *command, uint8_t data[CLAW_CAN_DLC])
{
  if ((command == NULL) || (data == NULL))
  {
    return;
  }

  data[0] = CLAW_PROTOCOL_VERSION;
  data[1] = command->sequence;
  data[2] = (uint8_t)command->opcode;
  data[3] = command->flags;
  write_i16_le(&data[4], command->target_cdeg);
  data[6] = command->speed_drpm;
  data[7] = ClawProtocol_Crc8(data, 7U);
}
