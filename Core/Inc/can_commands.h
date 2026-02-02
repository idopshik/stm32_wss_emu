/**
 * can_commands.h - VERSION 4.3 (SIMPLIFIED)
 * 
 * Только 3 режима:
 * 0x01 = RPM MODE
 * 0x02 = FIXED MODE
 * 0x03 = EXTERNAL MODE (бывший Hi-Z)
 */

#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H

#include "main.h"
#include "system_modes.h"
#include <stdint.h>

// ============================================
// CAN ИДЕНТИФИКАТОРЫ
// ============================================

#define CAN_SPECIAL_ID     0x003  // RPM данные (от ECU)
#define CAN_CMD_ID         0x004  // Командные сообщения (к STM32)
#define CAN_STATUS_ID      0x006  // Ответы статуса (от STM32)



// ============================================
// ФОРМАТ КОМАНД
// ============================================

#define CMD_REQUEST_STATUS  0x07
/*
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CMD 0x01 - RPM MODE (динамический режим)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x01
Byte 1: Channel mask (0x0F = все 4 канала, 0xFF = тоже все)
Bytes 2-7: не используются

Эффект:
    - Система переходит в MODE_RPM_DYNAMIC
    - Ждёт сообщений CAN ID 0x003 с RPM данными
    - LED мигает ТОЛЬКО когда данные активно приходят
    - Если нет данных >500ms, LED гаснет

RPM данные (ID 0x003):
    Byte 0-1: FL (TIM1) - uint16_t BE
    Byte 2-3: FR (TIM2) - uint16_t BE
    Byte 4-5: RL (TIM3) - uint16_t BE
    Byte 6-7: RR (TIM4) - uint16_t BE

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CMD 0x02 - FIXED MODE (фиксированная частота)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x02
Byte 1: Channel mask
    0x0F = ALL_FOUR (все 4 канала)
    0x02 = ONLY_FR (только TIM2, 32-бит)
    
Bytes 2-5: Частота в миллигерцах (uint32_t LE)
    Пример: 2500.0 Hz → 2500000 mHz
    Пример: 100.001 Hz → 100001 mHz
    Диапазон: 1000-4500000 mHz (1 Hz - 4500 Hz)

Bytes 6-7: Reserved

Эффект:
    - Система переходит в MODE_FIXED_FREQUENCY
    - Все таймеры работают на заданной частоте
    - LED постоянно мигает 500ms

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CMD 0x03 - EXTERNAL MODE (GPIO High-Z)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x03
Bytes 1-7: не используются

Эффект:
    - Система переходит в MODE_EXTERNAL_SIGNAL
    - Все GPIO переводятся в INPUT (High-Z)
    - SSR включается (PB10 = HIGH) для питания оптопары
    - Внешний сигнал может управлять GPIO
    - LED медленно мигает 2500ms

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
STATUS (0x006) - Автоматический ответ после команды
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: Текущий режим (1=RPM, 2=FIXED, 3=EXTERNAL)
Byte 1: Channel mask
Bytes 2-3: Частота ×5 (uint16_t LE) - только для FIXED режима
Bytes 4-5: Reserved
Bytes 6-7: Uptime в секундах (uint16_t LE)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ERROR (0x006) - Ошибка выполнения команды
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0xFF
Byte 1: Код ошибки
    0x01 = Неизвестная команда
    0x02 = Неверные параметры
    0x03 = Недопустимая маска каналов
Bytes 2-7: Reserved
*/

// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================

extern uint8_t can_tx_status_pending;
extern uint8_t can_tx_error_pending;
extern uint8_t can_tx_error_code;

extern void my_printf(const char *fmt, ...);

// ============================================
// ФУНКЦИИ
// ============================================

void process_can_command(uint8_t* data);
void send_system_status(void);
void send_error_response(uint8_t error_code);

// Расчёт PSC/ARR
void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr);
void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr);

#endif // CAN_COMMANDS_H
