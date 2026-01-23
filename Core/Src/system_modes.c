#include "system_modes.h"
#include <stdio.h>

// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================

system_state_t g_system_state;

// Имена режимов для отладки
const char* mode_names[] = {
    [MODE_BOOT] = "BOOT",
    [MODE_RPM_DYNAMIC] = "RPM_DYNAMIC",
    [MODE_FIXED_FREQUENCY] = "FIXED_FREQ",
    [MODE_PWM] = "PWM",
    [MODE_ANALOG_FOLLOW] = "ANALOG_FOLLOW",
    [MODE_DISABLED] = "DISABLED",
    [MODE_ERROR] = "ERROR"
};

// ============================================
// ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ
// ============================================

void system_init_modes(void)
{
    HAL_Delay(100);  // Ждем инициализации периферии
    printf("[SYSTEM] Initializing modes...\n");
    
    // Начальные значения
    g_system_state.current_mode = MODE_BOOT;
    g_system_state.channel_mask = 0x0F;  // Все 4 канала активны
    g_system_state.target_frequency_hz = 0;
    g_system_state.pwm_duty_percent = 50;
    
    // Инициализация таймеров значениями по умолчанию
    for(int i = 0; i < 4; i++) {
        g_system_state.psc_values[i] = 24;  // Значение из существующего кода
        g_system_state.arr_values[i] = 59999;
    }
    
    g_system_state.analog_signal_present = 0;
    g_system_state.eeprom_saved = 0;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    g_system_state.led_last_toggle_time = 0;
    g_system_state.led_state = 0;
    
    printf("[SYSTEM] Initialized. Mode: %s\n", 
           get_mode_name(g_system_state.current_mode));
}

// ============================================
// ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ
// ============================================

void system_switch_mode(operation_mode_t new_mode)
{
    operation_mode_t old_mode = g_system_state.current_mode;
    
    // Проверка валидности режима
    if(new_mode > MODE_DISABLED && new_mode != MODE_ERROR) {
        printf("[ERROR] Invalid mode: %d\n", new_mode);
        return;
    }
    
    printf("[SYSTEM] Switching mode: %s -> %s\n", 
           get_mode_name(old_mode), 
           get_mode_name(new_mode));
    
    // Действия при выходе из старого режима
    switch(old_mode) {
        case MODE_RPM_DYNAMIC:
            // Останавливаем динамическое обновление RPM
            printf("[SYSTEM] Exiting RPM dynamic mode\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            // Фиксированная частота уже установлена - ничего не делаем
            break;
            
        case MODE_ANALOG_FOLLOW:
            // Останавливаем отслеживание аналогового сигнала
            printf("[SYSTEM] Exiting analog follow mode\n");
            break;
            
        default:
            break;
    }
    
    // Устанавливаем новый режим
    g_system_state.current_mode = new_mode;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    // Действия при входе в новый режим
    switch(new_mode) {
        case MODE_BOOT:
            // Быстрое мигание LED
            printf("[SYSTEM] Entering BOOT mode\n");
            break;
            
        case MODE_RPM_DYNAMIC:
            // Включаем обработку CAN сообщений RPM
            printf("[SYSTEM] Entering RPM dynamic mode\n");
            printf("[SYSTEM] Waiting for CAN ID 0x003 RPM data...\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] Entering fixed frequency mode\n");
            if(g_system_state.target_frequency_hz == 0) {
                printf("[WARNING] Target frequency not set! Using 1 Hz\n");
                g_system_state.target_frequency_hz = 1;
            }
            // Здесь позже будет установка частоты на таймеры
            break;
            
        case MODE_PWM:
            printf("[SYSTEM] Entering PWM mode\n");
            printf("[SYSTEM] Freq: %lu Hz, Duty: %d%%\n",
                   g_system_state.target_frequency_hz,
                   g_system_state.pwm_duty_percent);
            break;
            
        case MODE_ANALOG_FOLLOW:
            printf("[SYSTEM] Entering analog follow mode\n");
            printf("[SYSTEM] Waiting for analog signal...\n");
            break;
            
        case MODE_DISABLED:
            printf("[SYSTEM] Entering disabled mode\n");
            // Останавливаем все таймеры
            for(int i = 0; i < 4; i++) {
                if(is_channel_active(i)) {
                    printf("[SYSTEM] Stopping timer %d\n", i+1);
                    // Здесь позже будет остановка таймеров
                }
            }
            break;
            
        case MODE_ERROR:
            printf("[SYSTEM] Entering ERROR mode\n");
            break;
    }
    
    // Сбрасываем время мигания LED
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    
    // Гасим LED для начала нового цикла мигания
    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
}

// ============================================
// УТИЛИТНЫЕ ФУНКЦИИ
// ============================================

const char* get_mode_name(operation_mode_t mode)
{
    if(mode <= MODE_DISABLED) {
        return mode_names[mode];
    }
    return "UNKNOWN";
}

void system_print_status(void)
{
    printf("\n=== SYSTEM STATUS ===\n");
    printf("Mode: %s\n", get_mode_name(g_system_state.current_mode));
    printf("Channels: ");
    for(int i = 0; i < 4; i++) {
        printf("%c", is_channel_active(i) ? '1' : '0');
    }
    printf(" (mask: 0x%02X)\n", g_system_state.channel_mask);
    
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY || 
       g_system_state.current_mode == MODE_PWM) {
        printf("Target freq: %lu Hz\n", g_system_state.target_frequency_hz);
    }
    
    if(g_system_state.current_mode == MODE_PWM) {
        printf("PWM duty: %d%%\n", g_system_state.pwm_duty_percent);
    }
    
    if(g_system_state.current_mode == MODE_ANALOG_FOLLOW) {
        printf("Analog signal: %s\n", 
               g_system_state.analog_signal_present ? "PRESENT" : "ABSENT");
    }
    
    printf("Last CAN cmd: %lu ms ago\n", 
           HAL_GetTick() - g_system_state.last_can_command_time);
    printf("EEPROM saved: %s\n", g_system_state.eeprom_saved ? "YES" : "NO");
    printf("===================\n");
}

void set_channel_active(uint8_t channel_num, uint8_t active)
{
    if(channel_num >= 4) return;
    
    if(active) {
        g_system_state.channel_mask |= (1 << channel_num);
    } else {
        g_system_state.channel_mask &= ~(1 << channel_num);
    }
    
    printf("[SYSTEM] Channel %d: %s\n", 
           channel_num + 1, active ? "ACTIVE" : "INACTIVE");
}

uint8_t is_channel_active(uint8_t channel_num)
{
    if(channel_num >= 4) return 0;
    return (g_system_state.channel_mask >> channel_num) & 0x01;
}

void set_all_channels_active(uint8_t active)
{
    g_system_state.channel_mask = active ? 0x0F : 0x00;
    printf("[SYSTEM] All channels: %s\n", active ? "ACTIVE" : "INACTIVE");
}
