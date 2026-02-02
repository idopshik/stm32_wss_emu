/**
 * system_modes.h - VERSION 4.2
 * 
 * Типы и структуры для управления режимами
 */

#ifndef SYSTEM_MODES_H
#define SYSTEM_MODES_H

#include <stdint.h>

// ============================================
// РЕЖИМЫ РАБОТЫ (3 шт)
// ============================================

typedef enum {
    MODE_BOOT = 0x00,               // Загрузка, ждёт команды
    MODE_RPM_DYNAMIC = 0x01,        // Динамический сигнал из CAN 0x003
    MODE_FIXED_FREQUENCY = 0x02,    // Статичная частота
    MODE_EXTERNAL_SIGNAL = 0x03     // Внешний сигнал (GPIO HIGH-Z, SSR ON)
} operation_mode_t;

// ============================================
// СТРУКТУРА СОСТОЯНИЯ СИСТЕМЫ
// ============================================

typedef struct {
    operation_mode_t current_mode;
    
    uint8_t channel_mask;           // Маска активных каналов (0x0F = все)
    uint32_t target_frequency_hz;   // Целевая частота в Гц
    uint32_t target_frequency_mhz;  // Целевая частота в миллигерцах
    
    // PSC и ARR для таймеров
    uint16_t psc_values[4];         // Prescaler для TIM1-4
    uint16_t arr_values[4];         // ARR для 16-бит таймеров
    uint32_t arr_values_32bit[4];   // ARR для 32-бит (TIM2)
    
    // Время и индикация
    uint32_t last_can_command_time;
    uint32_t boot_time;
    uint32_t led_last_toggle_time;
    uint8_t led_state;
    
    // LED BOOT вспышка
    uint8_t led_boot_flashed;
    uint32_t led_boot_start_time;
    
    // RPM режим: активен ли сигнал
    uint8_t rpm_signal_active;      // 1 = есть сигнал, 0 = нет
    
} system_state_t;

// ============================================
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// ============================================

extern system_state_t g_system_state;

// ============================================
// ФУНКЦИИ
// ============================================

void system_init_modes(void);
void system_switch_mode(operation_mode_t new_mode);
const char* get_mode_name(operation_mode_t mode);
uint32_t system_get_uptime_seconds(void);
void update_system_indicators(void);
void set_channel_active(uint8_t channel_num, uint8_t active);

#endif /* SYSTEM_MODES_H */
