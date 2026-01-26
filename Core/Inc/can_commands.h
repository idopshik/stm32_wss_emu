/**
 * can_commands.h - VERSION 2
 * 
 * Добавлено:
 * - CMD_SET_HI_IMPEDANCE (0x06) - безопасный режим для внешних сигналов
 * - Улучшенный формат статуса 0x006
 */

#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H

#include "main.h"
#include "system_modes.h"
#include <stdint.h>

// Уровни дебага
#define DEBUG_CAN_TX    0  // 0 = выключено, 1 = включено
#define DEBUG_CAN_ERROR 0  // 0 = выключено, 1 = включено
                           //
// ============================================
// CAN ID
// ============================================

#define DEBUG_CAN_COMMANDS 0  // 0 = выключено, 1 = включено
                              //
#define CAN_CMD_ID 0x004      // Командные сообщения
#define CAN_STATUS_ID 0x006   // Ответы статуса
                              
// Команды
#define CMD_STOP_ALL 0x00
#define CMD_RPM_MODE 0x01
#define CMD_FIXED_FREQ 0x02
#define CMD_ANALOG_FOLLOW 0x03
#define CMD_STATUS_REQUEST 0x07

// Объявления CAN переменных из main.c
extern volatile uint8_t can_tx_status_pending;
extern volatile uint8_t can_tx_error_pending;
extern uint8_t can_tx_error_code;

// Прототипы функций
void process_can_command(uint8_t* data);
void send_system_status(void);
void send_error_response(uint8_t error_code);
uint32_t get_timer_frequency(TIM_TypeDef* TIMx);

// ============================================
// КОДЫ КОМАНД (Байт 0)
// ============================================

typedef enum {
    CMD_DISABLE_OUTPUT = 0x00,      // Выключить выходы
    CMD_SET_RPM_MODE = 0x01,        // Перейти в RPM режим
    CMD_SET_FIXED_FREQ = 0x02,      // Установить фиксированную частоту
    CMD_SET_PWM_MODE = 0x03,        // Установить ШИМ режим
    CMD_SET_ANALOG_FOLLOW = 0x04,   // Режим следования аналоговому сигналу
    CMD_SET_HI_IMPEDANCE = 0x06,    // Hi-Z режим (защита от внешних сигналов)
    CMD_REQUEST_STATUS = 0x07       // Запросить статус системы
} can_command_t;

// ============================================
// ФОРМАТ СТАТУСА 0x006
// ============================================

/*
Byte 0: Режим работы
    0x00 = DISABLED
    0x01 = RPM_DYNAMIC
    0x02 = FIXED_FREQUENCY
    0x03 = PWM
    0x04 = ANALOG_FOLLOW
    0x05 = BOOT
    0x10 = HI_IMPEDANCE
    0xFF = ERROR

Byte 1: Channel mask (биты 0-3 = каналы 1-4)

Byte 2-3: Частота / 10 Hz (little-endian uint16_t)
    Пример: 100 Hz → 0x000A (10 × 10)
    Пример: 1000 Hz → 0x0064 (100 × 10)

Byte 4: PWM Duty % (0-100), или 0 если не PWM режим

Byte 5: Флаги состояния
    Bit 0: Analog signal present (1=да, 0=нет)
    Bit 1: Hi-Z mode active (1=да, 0=нет)
    Bit 2-7: reserved

Byte 6-7: Uptime в секундах (uint16_t little-endian)
*/

// ============================================
// ФУНКЦИИ ОБРАБОТКИ CAN КОМАНД
// ============================================

void process_can_command(uint8_t* data);
void send_system_status(void);
void send_error_response(uint8_t error_code);
void calculate_optimal_psc_arr(uint32_t freq_hz, uint16_t *psc, uint16_t *arr);
void set_fixed_frequency_to_timers(uint32_t freq_hz, uint16_t psc, uint16_t arr, uint8_t channel_mask);

// Новые функции для Hi-Z режима
void enter_hi_impedance_mode(void);
void exit_hi_impedance_mode(void);

#endif // CAN_COMMANDS_H
