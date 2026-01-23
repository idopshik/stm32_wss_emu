#include "eeprom_handler.h"
#include "main.h"
#include <string.h>

// Объявляем my_printf
extern void my_printf(const char *fmt, ...);

// Внешние переменные
extern system_state_t g_system_state;

// Локальная копия конфигурации
static eeprom_config_t eeprom_config;

// ============================================
// ИНИЦИАЛИЗАЦИЯ EEPROM
// ============================================

void eeprom_init(void)
{
    my_printf("[EEPROM] Initializing...\n");
    
    // Пробуем загрузить сохраненную конфигурацию
    if(eeprom_load_saved_state()) {
        my_printf("[EEPROM] Loaded saved configuration\n");
        my_printf("[EEPROM] Mode: %d, Freq: %lu Hz, Duty: %d%%\n",
                  eeprom_config.saved_mode,
                  eeprom_config.saved_frequency,
                  eeprom_config.saved_duty);
    } else {
        my_printf("[EEPROM] No saved configuration or CRC error\n");
        // Инициализируем дефолтными значениями
        eeprom_config.magic = EEPROM_MAGIC_NUMBER;
        eeprom_config.saved_mode = MODE_RPM_DYNAMIC;
        eeprom_config.saved_frequency = 0;
        eeprom_config.saved_duty = 50;
        eeprom_config.saved_channels = 0x0F;
        eeprom_config.crc32 = eeprom_calculate_crc(&eeprom_config);
    }
}

// ============================================
// СОХРАНЕНИЕ ТЕКУЩЕГО СОСТОЯНИЯ
// ============================================

void eeprom_save_current_state(void)
{
    my_printf("[EEPROM] Saving current state...\n");
    
    // Заполняем структуру текущим состоянием
    eeprom_config.magic = EEPROM_MAGIC_NUMBER;
    eeprom_config.saved_mode = g_system_state.current_mode;
    eeprom_config.saved_frequency = g_system_state.target_frequency_hz;
    eeprom_config.saved_duty = g_system_state.pwm_duty_percent;
    eeprom_config.saved_channels = g_system_state.channel_mask;
    
    // Вычисляем CRC
    eeprom_config.crc32 = eeprom_calculate_crc(&eeprom_config);
    
    // Сохраняем в EEPROM
    uint32_t data_addr = (uint32_t)&eeprom_config;
    uint32_t eeprom_addr = EEPROM_CONFIG_ADDR;
    
    // Разблокируем EEPROM
    HAL_FLASHEx_DATAEEPROM_Unlock();
    
    // Пишем данные (по 32 бита за операцию)
    for(uint32_t i = 0; i < sizeof(eeprom_config); i += 4) {
        uint32_t data_word;
        memcpy(&data_word, (void*)(data_addr + i), 4);
        
        // Используем правильное имя константы
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 
                                          eeprom_addr + i, 
                                          data_word) != HAL_OK) {
            my_printf("[EEPROM] Error writing at address 0x%08lX\n", eeprom_addr + i);
        }
    }
    
    // Блокируем EEPROM
    HAL_FLASHEx_DATAEEPROM_Lock();
    
    // Проверяем запись
    eeprom_load_saved_state();
    
    my_printf("[EEPROM] State saved successfully\n");
}

// ============================================
// ЗАГРУЗКА СОХРАНЕННОГО СОСТОЯНИЯ
// ============================================

uint8_t eeprom_load_saved_state(void)
{
    // Читаем данные из EEPROM
    memcpy(&eeprom_config, (void*)EEPROM_CONFIG_ADDR, sizeof(eeprom_config));
    
    // Проверяем magic number
    if(eeprom_config.magic != EEPROM_MAGIC_NUMBER) {
        my_printf("[EEPROM] Invalid magic number: 0x%08lX\n", eeprom_config.magic);
        return 0;
    }
    
    // Проверяем CRC
    uint32_t calculated_crc = eeprom_calculate_crc(&eeprom_config);
    if(calculated_crc != eeprom_config.crc32) {
        my_printf("[EEPROM] CRC error: stored=0x%08lX, calculated=0x%08lX\n",
                  eeprom_config.crc32, calculated_crc);
        return 0;
    }
    
    return 1;  // Успешно
}

// ============================================
// ВОССТАНОВЛЕНИЕ СОХРАНЕННОГО РЕЖИМА
// ============================================

void eeprom_restore_state(void)
{
    if(eeprom_load_saved_state()) {
        my_printf("[EEPROM] Restoring saved state...\n");
        
        // Восстанавливаем состояние системы
        g_system_state.current_mode = eeprom_config.saved_mode;
        g_system_state.target_frequency_hz = eeprom_config.saved_frequency;
        g_system_state.pwm_duty_percent = eeprom_config.saved_duty;
        g_system_state.channel_mask = eeprom_config.saved_channels;
        g_system_state.eeprom_saved = 1;
        
        // Применяем восстановленный режим
        system_switch_mode(eeprom_config.saved_mode);
        
        my_printf("[EEPROM] State restored: mode=%d\n", eeprom_config.saved_mode);
    }
}

// ============================================
// ФАБРИЧНЫЙ СБРОС
// ============================================

void eeprom_factory_reset(void)
{
    my_printf("[EEPROM] Performing factory reset...\n");
    
    // Разблокируем EEPROM
    HAL_FLASHEx_DATAEEPROM_Unlock();
    
    // Стираем всю EEPROM (записываем 0)
    for(uint32_t addr = EEPROM_CONFIG_ADDR; 
        addr < EEPROM_CONFIG_ADDR + sizeof(eeprom_config); 
        addr += 4) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, addr, 0x00000000);
    }
    
    // Блокируем EEPROM
    HAL_FLASHEx_DATAEEPROM_Lock();
    
    // Сбрасываем структуру
    memset(&eeprom_config, 0, sizeof(eeprom_config));
    
    my_printf("[EEPROM] Factory reset complete\n");
}

// ============================================
// ВЫЧИСЛЕНИЕ CRC32 (упрощенная версия)
// ============================================

uint32_t eeprom_calculate_crc(const eeprom_config_t* config)
{
    uint32_t crc = 0xFFFFFFFF;
    uint8_t* data = (uint8_t*)config;
    
    // Не включаем поле crc32 в расчет
    size_t data_size = sizeof(eeprom_config_t) - sizeof(config->crc32);
    
    for(size_t i = 0; i < data_size; i++) {
        crc ^= data[i];
        for(int j = 0; j < 8; j++) {
            if(crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}
