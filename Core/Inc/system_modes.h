/**
 * system_modes.h - VERSION 3
 * 
 * Изменения:
 * - Добавлено arr_values_32bit для TIM2 (32-битный режим)
 * - Добавлено target_frequency_mhz для более точной работы
 * - УДАЛЕНЫ аналоговые режимы
 */

#ifndef SYSTEM_MODES_H
#define SYSTEM_MODES_H
#include "main.h"
#include <stdint.h>

#define DEBUG_SYSTEM

// ============================================
// РЕЖИМЫ РАБОТЫ СИСТЕМЫ
// ============================================

typedef enum {
    MODE_BOOT = 0,              // Загрузка
    MODE_RPM_DYNAMIC = 1,       // Динамический RPM (управление по CAN 0x003)
    MODE_FIXED_FREQUENCY = 2,   // Фиксированная частота (ALL_FOUR или ONLY_FR)
    MODE_DISABLED = 5,          // Выключено (таймеры остановлены)
    MODE_HI_IMPEDANCE = 0x10,   // Hi-Z режим (GPIO = input, SSR активен)
    MODE_ERROR = 0xFF           // Ошибка
} operation_mode_t;

// ============================================
// СТРУКТУРА СОСТОЯНИЯ СИСТЕМЫ
// ============================================

typedef struct {
    // Текущее состояние режима
    operation_mode_t current_mode;
    uint8_t pending_hi_z;              // Флаг для отложенного перехода в Hi-Z
    uint8_t channel_mask;              // Биты: 0=TIM1, 1=TIM2, 2=TIM3, 3=TIM4
    
    // Параметры частоты
    uint32_t target_frequency_hz;      // Целая часть частоты (для совместимости)
    uint32_t target_frequency_mhz;     // Частота в миллигерцах (от CAN)
    
    // Параметры таймеров
    uint16_t psc_values[4];            // PSC для каждого таймера [0]=TIM1...
    uint16_t arr_values[4];            // ARR для каждого таймера (16-бит)
    uint32_t arr_values_32bit[4];      // ARR для 32-бит режима (только TIM2 использует)
    
    // PWM параметры (если потребуются позже)
    uint8_t pwm_duty_percent;
    
    // Флаги состояния
    uint8_t analog_signal_present;     // Зарезервировано (не используется)
    uint8_t hi_impedance_active;       // 1 = Hi-Z режим активен
    
    // Для отслеживания активности
    uint32_t last_can_command_time;    // Время последней CAN команды
    uint32_t boot_time;                // Время загрузки (для uptime)
    
    // Для LED индикации
    uint32_t led_last_toggle_time;
    uint8_t led_state;
    uint8_t rpm_mode_active;           // Флаг: активен ли приём RPM данных
    
    // Для MODE_BOOT LED (одна вспышка 1500ms)
    uint8_t led_boot_flashed;          // 1 = LED уже включали в BOOT режиме
    uint32_t led_boot_start_time;      // Время когда включили LED в BOOT
    
} system_state_t;

// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (extern)
// ============================================

extern system_state_t g_system_state;
extern const char* mode_names[];

// ============================================
// ФУНКЦИИ УПРАВЛЕНИЯ РЕЖИМАМИ
// ============================================

void system_init_modes(void);
void system_switch_mode(operation_mode_t new_mode);
const char* get_mode_name(operation_mode_t mode);
void system_print_status(void);
uint32_t system_get_uptime_seconds(void);


// Hi-Z режим
void enter_hi_impedance_mode(void);
void exit_hi_impedance_mode(void);

// Утилиты для работы с каналами
void set_channel_active(uint8_t channel_num, uint8_t active);
uint8_t is_channel_active(uint8_t channel_num);
void set_all_channels_active(uint8_t active);

#endif // SYSTEM_MODES_H
