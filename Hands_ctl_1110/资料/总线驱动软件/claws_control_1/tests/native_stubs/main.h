#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stdlib.h>
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef struct { unsigned unused; } TIM_HandleTypeDef;
static GPIO_TypeDef ports[5];
static uint32_t test_tick;
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define HAL_OK 0
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_3 3
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((void)(t),(void)(c),(void)(v))
static uint32_t HAL_GetTick(void) { return test_tick; }
static int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) {
  (void)p;
  return pin == (1U<<6) || pin == (1U<<9) || pin == (1U<<12);
}
static void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, int v) {
  (void)p; (void)pin; (void)v;
}
#endif
