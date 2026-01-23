#ifndef SYSTEM_MODES_H
#define SYSTEM_MODES_H

#include "main.h"
#include <stdint.h>

// ============================================
// РЕЖИМЫ РАБОТЫ СИСТЕМЫ
// ============================================

typedef enum {
    MODE_BOOT = 0,              // Загрузка (быстрое мигание 5 Гц)
    MODE_RPM_DYNAMIC = 1,       // Динамический RPM (2 Гц)
    MODE_FIXED_FREQUENCY = 2,   // Фиксированная частота (1 Гц)
    MODE_PWM = 3,               // ШИМ режим (0.5 Гц)
    MODE_ANALOG_FOLLOW = 4,     // Следование аналоговому сигналу
    MODE_DISABLED = 5,          // Выключено (LED выкл)
    MODE_ERROR = 0xFF           // Ошибка (10 Гц)
} operation_mode_t;

// ============================================
// СТРУКТУРА СОСТОЯНИЯ СИСТЕМЫ
// ============================================

typedef struct {
    // Текущее состояние
    operation_mode_t current_mode;
    uint8_t channel_mask;          // Биты: 0=TIM1, 1=TIM2, 2=TIM3, 3=TIM4
    uint32_t target_frequency_hz;  // Целевая частота (для режимов 2,3)
    uint16_t psc_values[4];        // PSC для каждого таймера [0]=TIM1...
    uint16_t arr_values[4];        // ARR для каждого таймера
    uint8_t pwm_duty_percent;      // Скважность ШИМ (0-100%)
    
    // Флаги состояния
    uint8_t analog_signal_present; // 1 = аналоговый сигнал есть
    
    // Для отслеживания активности
    uint32_t last_can_command_time; // Время последней CAN команды
    
    // Для индикации
    uint32_t led_last_toggle_time;
    uint8_t led_state;
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

// Утилиты для работы с каналами
void set_channel_active(uint8_t channel_num, uint8_t active);
uint8_t is_channel_active(uint8_t channel_num);
void set_all_channels_active(uint8_t active);

#endif // SYSTEM_MODES_H
