#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H

#include "main.h"
#include "system_modes.h"
#include <stdint.h>

// ============================================
// CAN ID ДЛЯ КОМАНД
// ============================================

#define CAN_CMD_ID 0x004      // Командные сообщения
#define CAN_STATUS_ID 0x006   // Ответы статуса

// ============================================
// КОДЫ КОМАНД (Байт 0)
// ============================================

typedef enum {
    CMD_DISABLE_OUTPUT = 0x00,      // Выключить выходы
    CMD_SET_RPM_MODE = 0x01,        // Перейти в RPM режим
    CMD_SET_FIXED_FREQ = 0x02,      // Установить фиксированную частоту
    CMD_SET_PWM_MODE = 0x03,        // Установить ШИМ режим
    CMD_SET_ANALOG_FOLLOW = 0x04,   // Режим следования аналоговому сигналу
    CMD_SAVE_TO_EEPROM = 0x05,      // Сохранить текущий режим в EEPROM
    CMD_FACTORY_RESET = 0x06,       // Сброс EEPROM к заводским настройкам
    CMD_REQUEST_STATUS = 0x07       // Запросить статус системы
} can_command_t;

// ============================================
// ФУНКЦИИ ОБРАБОТКИ CAN КОМАНД
// ============================================

void process_can_command(uint8_t* data);
void send_system_status(void);
void send_error_response(uint8_t error_code);
void set_fixed_frequency_to_timers(uint32_t freq_hz, uint16_t psc, uint8_t channel_mask);

#endif // CAN_COMMANDS_H
