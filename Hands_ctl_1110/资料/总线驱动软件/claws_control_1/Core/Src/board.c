#include "board.h"
#include "claw_control.h"
#include "claw_can_config.h"

volatile uint32_t board_tx_drops, board_can_errors;
static volatile uint32_t rx_drops;
static volatile uint8_t pending_fault, inhibited;
static uint8_t reset_flags;
static const uint32_t channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
#define ENABLE_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
#define BRAKE_PINS (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5)
#define HALL_PINS (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14)
#define QUEUE_SIZE 32u
static can_frame_t rx_queue[QUEUE_SIZE], tx_queue[QUEUE_SIZE];
static volatile uint8_t rx_head, rx_tail;
static uint8_t tx_head, tx_tail;
static uint32_t tx_busy_since;
static bool tx_was_busy;
static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static void check(HAL_StatusTypeDef s) { if (s != HAL_OK) Board_Panic(); }

void Board_EarlySafe(void) {
    reset_flags = (uint8_t)(RCC->CSR >> 24);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIOA->BSRR = ENABLE_PINS | BRAKE_PINS; /* nENABLE=1, nBRAKE=1 before output mode */
    GPIO_InitTypeDef g = {0};
    g.Pin = ENABLE_PINS | BRAKE_PINS | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    GPIOC->BSRR = GPIO_PIN_13;
    g.Pin = GPIO_PIN_13; HAL_GPIO_Init(GPIOC, &g);
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) { ClawControl_HallEdgeIRQ(pin); }
void Board_PreparePWM(void) {
    /* Refuse unsafe clock/polarity drift after editing the CubeMX configuration.
       nENABLE is still GPIO HIGH here; MspPostInit connects AF only after return. */
    if (HAL_RCC_GetHCLKFreq() != 72000000u || HAL_RCC_GetPCLK2Freq() != 72000000u ||
        htim1.Init.Prescaler != 0u || htim1.Init.Period != APP_PWM_PERIOD - 1u ||
        TIM1->CCR1 || TIM1->CCR2 || TIM1->CCR3 ||
        (TIM1->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P)) !=
            (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P) ||
        (TIM1->CR2 & (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3)) !=
            (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3)) Board_Panic();
    for (unsigned i=0; i<CLAW_MOTOR_COUNT; ++i)
        check(HAL_TIM_PWM_Start(&htim1, channels[i]));
}
void Board_KillOutputs(void) {
    uint32_t p = lock();
    inhibited = 1;
    GPIOA->BSRR = ENABLE_PINS | BRAKE_PINS;
    /* Directly take ENABLE pins out of AF; even pending CCR preload cannot re-enable. */
    GPIOA->CRH = (GPIOA->CRH & ~0xFFFu) | 0x222u;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    unlock(p);
}
bool Board_ReleaseOutputs(void) {
    uint32_t p = lock();
    if (pending_fault || (CAN1->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF))) { unlock(p); return false; }
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->BDTR |= TIM_BDTR_MOE;
    GPIOA->CRH = (GPIOA->CRH & ~0xFFFu) | 0xBBBu;
    inhibited = 0;
    unlock(p);
    return true;
}
uint16_t Board_SetPWM(unsigned i, uint16_t duty) {
    if (i >= CLAW_MOTOR_COUNT) return 0;
    uint32_t p = lock();
    if (inhibited) { unlock(p); return 0; }
    if (duty > CLAW_PWM_SAFE_MAX_PERMILLE) duty = CLAW_PWM_SAFE_MAX_PERMILLE;
    uint32_t count = ((uint32_t)duty * APP_PWM_PERIOD) / 1000u;
    /* LOW-active ENABLE: CCR=0 -> constant HIGH -> output disabled. */
    if (duty == 0u) {
        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, channels[i]);
        __HAL_TIM_SET_COMPARE(&htim1, channels[i], 0);
        __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, channels[i]);
    } else __HAL_TIM_SET_COMPARE(&htim1, channels[i], count);
    unlock(p);
    return duty;
}
static void signal_fault(uint8_t f) {
    pending_fault |= f;
    Board_KillOutputs();
}
static void can_init(void) {
    /* CubeMX owns bit timing; app_config.h is an explicit compatibility guard. */
    if (HAL_RCC_GetPCLK1Freq() != 36000000u ||
        hcan.Init.Prescaler != 36000000u / (18u * APP_CAN_BITRATE) ||
        hcan.Init.TimeSeg1 != CAN_BS1_15TQ || hcan.Init.TimeSeg2 != CAN_BS2_2TQ ||
        hcan.Init.SyncJumpWidth != CAN_SJW_1TQ || hcan.Init.Mode != CAN_MODE_NORMAL ||
        hcan.Init.AutoBusOff != ENABLE || hcan.Init.AutoRetransmission != ENABLE ||
        hcan.Init.ReceiveFifoLocked != ENABLE || hcan.Init.TransmitFifoPriority != ENABLE)
        Board_Panic();
    for (unsigned bank=0; bank<1; ++bank) {
        CAN_FilterTypeDef filter = {0};
        filter.FilterBank = bank; filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = CLAW_CAN_ID_COMMAND << 5;
        filter.FilterMaskIdHigh = 0xFFE0u;
        filter.FilterMaskIdLow = 6u; /* Match IDE=0 and RTR=0 as well. */
        filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter.FilterActivation = ENABLE; filter.SlaveStartFilterBank = 14;
        check(HAL_CAN_ConfigFilter(&hcan, &filter));
    }
    check(HAL_CAN_Start(&hcan));
    check(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
        CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR));
}
bool Board_SetCanBaseId(uint16_t base, bool save) {
    CAN_FilterTypeDef filter = {0};
    if (base < CLAW_CAN_ID_MIN_BASE || base > CLAW_CAN_ID_MAX_BASE) return false;
    if (HAL_CAN_Stop(&hcan) != HAL_OK) return false;
    if (!ClawCanConfig_SetBase(base, save)) { (void)HAL_CAN_Start(&hcan); return false; }
    filter.FilterBank = 0; filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = CLAW_CAN_ID_COMMAND << 5;
    filter.FilterMaskIdHigh = 0xFFE0u; filter.FilterMaskIdLow = 6u;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK || HAL_CAN_Start(&hcan) != HAL_OK) return false;
    return true;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle) {
    /* Hardware FIFO has three slots. Bound each ISR so flood traffic cannot starve foreground. */
    for (unsigned budget=0; budget<3u && HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0); ++budget) {
        CAN_RxHeaderTypeDef h; can_frame_t f = {0};
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &h, f.data) != HAL_OK) { signal_fault(CLAW_FAULT_COMM_TIMEOUT); break; }
        f.id = (uint16_t)h.StdId; f.dlc = (uint8_t)h.DLC;
        f.extended = h.IDE != CAN_ID_STD; f.remote = h.RTR != CAN_RTR_DATA;
        f.timestamp = HAL_GetTick();
        uint8_t next = (uint8_t)((rx_head+1u) % QUEUE_SIZE);
        if (next == rx_tail) { ++rx_drops; signal_fault(CLAW_FAULT_COMM_TIMEOUT); continue; }
        rx_queue[rx_head] = f; __DMB(); rx_head = next;
    }
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle) {
    uint32_t e = HAL_CAN_GetError(handle);
    board_can_errors |= e;
    if (e & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_RX_FOV0)) {
        if (e & HAL_CAN_ERROR_RX_FOV0) ++rx_drops;
        signal_fault(CLAW_FAULT_COMM_TIMEOUT);
    }
    (void)HAL_CAN_ResetError(handle);
}
bool Board_CAN_Pop(can_frame_t *f) {
    uint32_t p = lock();
    if (rx_tail == rx_head) { unlock(p); return false; }
    *f = rx_queue[rx_tail]; rx_tail = (uint8_t)((rx_tail+1u) % QUEUE_SIZE);
    unlock(p); return true;
}
uint8_t Board_TakeFaults(void) {
    uint32_t p = lock(); uint8_t f = pending_fault; pending_fault = 0;
    if (f) rx_tail = rx_head; /* Do not execute old CLEAR/SET frames after a new fault. */
    unlock(p); return f;
}
bool Board_Send(uint16_t id, const uint8_t data[8]) {
    uint8_t next = (uint8_t)((tx_head+1u) % QUEUE_SIZE);
    if (next == tx_tail) { ++board_tx_drops; signal_fault(CLAW_FAULT_COMM_TIMEOUT); return false; }
    can_frame_t *f = &tx_queue[tx_head];
    f->id = id; f->timestamp = HAL_GetTick(); for (unsigned i=0; i<8u; ++i) f->data[i] = data[i];
    tx_head = next; return true;
}
void Board_CAN_Service(void) {
    uint32_t now = HAL_GetTick();
    uint32_t free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    if (free < 3u) {
        if (!tx_was_busy) { tx_busy_since = now; tx_was_busy = true; }
        if (now - tx_busy_since >= 100u) {
            /* Prevent forever-pending telemetry on an unacknowledged/disconnected bus. */
            (void)HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            tx_busy_since = now;
            signal_fault(CLAW_FAULT_COMM_TIMEOUT);
        }
    } else tx_was_busy = false;
    for (unsigned budget=0; budget<3u && tx_tail != tx_head; ++budget) {
        can_frame_t *f = &tx_queue[tx_tail];
        if (now - f->timestamp > APP_MAX_RX_AGE_MS) { ++board_tx_drops; tx_tail=(uint8_t)((tx_tail+1u)%QUEUE_SIZE); continue; }
        if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan) || (CAN1->ESR & CAN_ESR_BOFF)) break;
        CAN_TxHeaderTypeDef h = {0}; uint32_t mailbox;
        h.StdId=f->id; h.IDE=CAN_ID_STD; h.RTR=CAN_RTR_DATA; h.DLC=8;
        if (HAL_CAN_AddTxMessage(&hcan, &h, f->data, &mailbox) != HAL_OK) break;
        tx_tail=(uint8_t)((tx_tail+1u)%QUEUE_SIZE);
    }
}
void Board_Init(void) {
    /* Called after CubeMX GPIO/CAN/TIM/IWDG initialization. */
    __HAL_DBGMCU_FREEZE_IWDG();
    can_init();
}
uint8_t Board_ResetFlags(void) { return reset_flags; }
void Board_FeedWatchdog(void) { (void)HAL_IWDG_Refresh(&hiwdg); }
void Board_Panic(void) {
    Board_KillOutputs();
    GPIOC->BRR = GPIO_PIN_13;
    __disable_irq();
    while (1) { /* IWDG resets if already running. Remain off before watchdog setup. */ }
}
void HAL_RCC_CSSCallback(void) { Board_Panic(); }
