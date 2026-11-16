#ifndef BOARD_H
#define BOARD_H
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "app_config.h"
#include "claw_protocol.h"
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern IWDG_HandleTypeDef hiwdg;
typedef struct {
  uint16_t id;
  uint8_t dlc, data[8];
  bool extended, remote;
  uint32_t timestamp;
} can_frame_t;
void Board_EarlySafe(void);
void Board_Init(void);
void Board_PreparePWM(void);
uint16_t Board_SetPWM(unsigned motor, uint16_t duty);
void Board_KillOutputs(void);
bool Board_ReleaseOutputs(void);
void Board_CAN_Service(void);
bool Board_CAN_Pop(can_frame_t *frame);
bool Board_Send(uint16_t id, const uint8_t data[8]);
bool Board_SetCanBaseId(uint16_t base, bool save);
uint8_t Board_TakeFaults(void);
void Board_Panic(void);
void Board_FeedWatchdog(void);
#endif
