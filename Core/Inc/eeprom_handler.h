#ifndef EEPROM_HANDLER_H
#define EEPROM_HANDLER_H

#include "system_modes.h"
#include <stdint.h>

// Адреса в EEPROM (STM32 имеет 4KB EEPROM)
#define EEPROM_CONFIG_ADDR  0x08080000  // Начало EEPROM
#define EEPROM_MAGIC_NUMBER 0x55AA55AA

// Структура конфигурации для сохранения
typedef struct __packed {
    uint32_t magic;             // 0x55AA55AA для проверки
    uint8_t saved_mode;         // Сохраненный режим
    uint32_t saved_frequency;   // Частота (для фикс. и ШИМ режимов)
    uint8_t saved_duty;         // Скважность (для ШИМ)
    uint8_t saved_channels;     // Активные каналы
    uint32_t crc32;             // Контрольная сумма
} eeprom_config_t;

// Функции EEPROM
void eeprom_init(void);
void eeprom_save_current_state(void);
uint8_t eeprom_load_saved_state(void);
void eeprom_factory_reset(void);
uint32_t eeprom_calculate_crc(const eeprom_config_t* config);

#endif // EEPROM_HANDLER_H
