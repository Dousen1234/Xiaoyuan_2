#include "claw_can_config.h"
#include "claw_protocol.h"
#include "stm32f1xx_hal.h"

#define CLAW_CAN_CONFIG_ADDR 0x0800FC00UL
#define CLAW_CAN_CONFIG_MAGIC 0x43414E49UL
volatile uint16_t claw_can_id_base = CLAW_CAN_ID_DEFAULT_BASE;

static bool valid_base(uint16_t base) { return base >= CLAW_CAN_ID_MIN_BASE && base <= CLAW_CAN_ID_MAX_BASE; }

void ClawCanConfig_Init(void)
{
  const uint32_t *p = (const uint32_t *)CLAW_CAN_CONFIG_ADDR;
  if (p[0] == CLAW_CAN_CONFIG_MAGIC && (uint16_t)p[1] == (uint16_t)(~p[2]) && valid_base((uint16_t)p[1]))
    claw_can_id_base = (uint16_t)p[1];
  else claw_can_id_base = CLAW_CAN_ID_DEFAULT_BASE;
}

bool ClawCanConfig_SetBase(uint16_t base, bool save)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t error = 0U;
  if (!valid_base(base)) return false;
  claw_can_id_base = base;
  if (!save) return true;
  HAL_FLASH_Unlock();
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = CLAW_CAN_CONFIG_ADDR;
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK ||
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CLAW_CAN_CONFIG_ADDR, CLAW_CAN_CONFIG_MAGIC) != HAL_OK ||
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CLAW_CAN_CONFIG_ADDR + 4U, base) != HAL_OK ||
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CLAW_CAN_CONFIG_ADDR + 8U, (uint32_t)(uint16_t)(~base)) != HAL_OK)
  { HAL_FLASH_Lock(); return false; }
  HAL_FLASH_Lock();
  return true;
}
