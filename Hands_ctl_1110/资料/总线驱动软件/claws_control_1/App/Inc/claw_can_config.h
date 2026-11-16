#ifndef CLAW_CAN_CONFIG_H
#define CLAW_CAN_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
void ClawCanConfig_Init(void);
bool ClawCanConfig_SetBase(uint16_t base, bool save);
#endif
