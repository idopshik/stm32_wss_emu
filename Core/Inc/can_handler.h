// can_handler.h
#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can_handler.h"
#include "wheel_control.h"
#include "system_modes.h"
#include "can_commands.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
//
// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (из main.c)
// ============================================

extern  uint8_t can_tx_status_pending;
extern  uint8_t can_tx_error_pending;
extern uint8_t can_tx_error_code;

// ========== Переменные (объявлены в can_handler.c) ==========
extern uint8_t canRX[8];
extern uint8_t freshCanMsg;
extern uint8_t freshCanCmd;
extern uint8_t canCmdData[8];
extern uint32_t can_callback_count;

extern FDCAN_HandleTypeDef hfdcan1;
extern void my_printf(const char *fmt, ...);

// ========== CAN IDs ==========
#define CAN_SPECIAL_ID  0x003  // RPM данные (от ECU)
#define CAN_CMD_ID      0x004  // Команды управления ← БЫЛО MISSING!
#define CAN_STATUS_ID   0x006  // Статус системы (исходящее)

// ========== Функции ==========
void can_handler_init(void);
void can_process_in_main(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


// #define DEBUG_CAN          // Основная диагностика
// #define DEBUG_CAN_DETAIL   // Детальная диагностика (много вывода)

#endif



