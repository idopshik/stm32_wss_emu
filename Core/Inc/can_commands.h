/**
 * can_commands.h - VERSION 3
 * 
 * Изменения:
 * - Убраны аналоговые команды
 * - ТОЛЬКО: RPM, FIXED_FREQ, HI_Z, STATUS, DISABLE
 * - Частота передаётся в миллигерцах для точности до 3-го знака
 */

#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H

// ============================================
// НАСТРОЙКИ ОТЛАДКИ
// ============================================

// Раскомментируйте нужные дебаг-флаги:
#define DEBUG_CAN_COMMANDS    // Дебаг обработки команд
#define DEBUG_CAN_TX          // Дебаг отправки сообщений  
#define DEBUG_CAN_ERROR       // Дебаг ошибок
#define DEBUG_CALC            // Дебаг расчётов PSC/ARR

#include "main.h"
#include "system_modes.h"
#include <stdint.h>



// ============================================
// CAN ИДЕНТИФИКАТОРЫ
// ============================================

#define CAN_RPM_ID         0x003  // RPM данные (от ECU или ЭБУ)
#define CAN_CMD_ID         0x004  // Командные сообщения (к STM32)
#define CAN_STATUS_ID      0x006  // Ответы статуса (от STM32)





// ============================================
// ФОРМАТ КОМАНД
// ============================================

/*
CMD_SET_FIXED_FREQ (0x02) - Установить фиксированную частоту
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x02 (команда SET_FIXED_FREQ)

Byte 1: Channel mask
    0x0F = ALL_FOUR (все 4 таймера 16-бит)
    0x02 = ONLY_FR (только TIM2 32-бит, максимальная точность)
    Другие значения: ошибка

Byte 2-5: Частота в миллигерцах (uint32_t little-endian)
    Пример: 2500.0 Hz → 2500000 mHz → 0x264B5900
    Пример: 4500.123 Hz → 4500123 mHz → 0x449F4600
    Пример: 100.001 Hz → 100001 mHz → 0x86860100
    
    Диапазон: 1-4500000 mHz (0.001 Hz - 4500 Hz)
    Это позволяет представить частоту до 3-го знака после запятой!

Byte 6: Флаг автоматического расчёта (всегда 0x00)
    0x00 = STM32 сам считает оптимальные PSC/ARR
    0xFF = то же самое (для совместимости)

Byte 7: Резервировано (=0)

Поведение STM32:
    - Маска 0x0F:  расчёт для 16-битных, одинаковые PSC/ARR для всех
    - Маска 0x02:  расчёт для 32-битного TIM2, остальные останавливаются
    - Другая маска: ошибка 0x03

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

CMD_SET_RPM_MODE (0x01) - Режим RPM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x01
Byte 1: Channel mask (0xFF = все)
Bytes 2-7: не используются

STM32 ждёт команд CAN ID 0x003 с RPM значениями:
    Byte 0-1: FL (TIM1)   - uint16_t BE
    Byte 2-3: FR (TIM2)   - uint16_t BE
    Byte 4-5: RL (TIM3)   - uint16_t BE
    Byte 6-7: RR (TIM4)   - uint16_t BE

LED мигает 2Hz только когда данные активно приходят.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

CMD_SET_HI_IMPEDANCE (0x06) - Hi-Z режим
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: 0x06
Bytes 1-7: не используются

Эффект:
    - Останавливаются все таймеры
    - PA5 (FR) переводится в INPUT (Hi-Z)
    - SSR Control (PB10) устанавливается в HIGH
    - Внешний генератор может управлять PA5 через оптрон

Выход из Hi-Z: любая другая команда (0x00, 0x01, 0x02, 0x07)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STATUS (0x006) - Ответ на CMD_REQUEST_STATUS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Byte 0: Режим работы
Byte 1: Channel mask
Byte 2-3: Частота ×10 Hz (uint16_t LE)
Byte 4: Флаги (биты: 0=signal, 1=hi_z)
Byte 5-6: Uptime (uint16_t LE)
Byte 7: Reserved
*/

// ============================================
// КОДЫ КОМАНД (Byte 0 в CAN сообщении)
// ============================================

typedef enum {
    CMD_DISABLE_OUTPUT = 0x00,      // Выключить выходы
    CMD_SET_RPM_MODE = 0x01,        // Перейти в RPM режим
    CMD_SET_FIXED_FREQ = 0x02,      // Установить фиксированную частоту
    CMD_SET_HI_IMPEDANCE = 0x06,    // Hi-Z режим (защита от внешних сигналов)
    CMD_REQUEST_STATUS = 0x07       // Запросить статус системы
} can_command_t;

// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (из main.c)
// ============================================

extern volatile uint8_t can_tx_status_pending;
extern volatile uint8_t can_tx_error_pending;
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
