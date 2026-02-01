// can_handler.h
#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can_handler.h"
#include "wheel_control.h"
#include "system_modes.h"
#include "can_commands.h"
#include <string.h>
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern void my_printf(const char *fmt, ...);

// Переменные (объявлены в can_handler.c)
extern uint8_t canRX[8];
extern uint8_t freshCanMsg;
extern uint8_t freshCanCmd;
extern uint8_t canCmdData[8];

// Функции
void can_handler_init(void);
void can_process_in_main(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif
