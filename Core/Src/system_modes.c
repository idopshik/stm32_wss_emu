
/**
 * system_modes.c - VERSION 3
 * 
 * Изменения:
 * - Удалены все аналоговые режимы и функции
 * - Упрощена логика режимов (только RPM, FIXED, HI-Z, DISABLED)
 * - Добавлен флаг rpm_mode_active для LED логики
 */

#include "system_modes.h"
#include <stdio.h>

extern void my_printf(const char *fmt, ...);

system_state_t g_system_state;

const char* mode_names[] = {
    [MODE_BOOT] = "BOOT",
    [MODE_RPM_DYNAMIC] = "RPM_DYNAMIC",
    [MODE_FIXED_FREQUENCY] = "FIXED_FREQ",
    [MODE_DISABLED] = "DISABLED",
    [MODE_HI_IMPEDANCE] = "HI_IMPEDANCE",
    [MODE_ERROR] = "ERROR"
};

// ============================================
// ИНИЦИАЛИЗАЦИЯ РЕЖИМОВ
// ============================================

void system_init_modes(void)
{
    #if DEBUG_SYSTEM
    my_printf("[SYSTEM] Initializing modes...\n");
    #endif

    g_system_state.pending_hi_z = 0;
    g_system_state.current_mode = MODE_BOOT;
    g_system_state.channel_mask = 0x0F;
    g_system_state.target_frequency_hz = 0;
    g_system_state.target_frequency_mhz = 0;
    g_system_state.pwm_duty_percent = 50;
    
    for(int i = 0; i < 4; i++) {
        g_system_state.psc_values[i] = 24;
        g_system_state.arr_values[i] = 59999;
        g_system_state.arr_values_32bit[i] = 59999;
    }
    
    g_system_state.analog_signal_present = 0;
    g_system_state.hi_impedance_active = 0;
    g_system_state.last_can_command_time = HAL_GetTick();
    g_system_state.boot_time = HAL_GetTick();
    g_system_state.led_last_toggle_time = 0;
    g_system_state.led_state = 0;
    g_system_state.rpm_mode_active = 0;
    
    // Инициализация MODE_BOOT LED (одна вспышка 1500ms)
    g_system_state.led_boot_flashed = 0;
    g_system_state.led_boot_start_time = 0;
    
    #if DEBUG_SYSTEM
    my_printf("[SYSTEM] Initialized. Mode: %s\n", 
             get_mode_name(g_system_state.current_mode));
    #endif
}

// ============================================
// ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ
// ============================================

void system_switch_mode(operation_mode_t new_mode)
{
    operation_mode_t old_mode = g_system_state.current_mode;
    
    if(old_mode == new_mode) {
        #if DEBUG_SYSTEM
        my_printf("[SYSTEM] Already in mode: %s\n", get_mode_name(new_mode));
        #endif
        return;
    }
    
    #if DEBUG_SYSTEM
    my_printf("\n[SYSTEM] Mode transition: %s → %s\n", 
              get_mode_name(old_mode), 
              get_mode_name(new_mode));
    #endif
    
    // ============================================
    // ВЫХОД ИЗ СТАРОГО РЕЖИМА
    // ============================================
    
    // Специфичные действия при выходе
    switch(old_mode) {
        case MODE_RPM_DYNAMIC:
            g_system_state.rpm_mode_active = 0;
            #if DEBUG_SYSTEM
            my_printf("[SYSTEM] RPM mode ended\n");
            #endif
            break;
            
        case MODE_FIXED_FREQUENCY:
            #if DEBUG_SYSTEM
            my_printf("[SYSTEM] Exiting FIXED_FREQUENCY\n");
            #endif
            break;
            
        case MODE_HI_IMPEDANCE:
            #if DEBUG_SYSTEM
            my_printf("[SYSTEM] Exiting HI_IMPEDANCE\n");
            #endif
            break;
            
        default:
            break;
    }
    
    // Переключаемся
    g_system_state.current_mode = new_mode;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    // ============================================
    // ВХОД В НОВЫЙ РЕЖИМ
    // ============================================
    
    #if DEBUG_SYSTEM
    switch(new_mode) {
        case MODE_BOOT:
            my_printf("[SYSTEM] BOOT mode\n");
            break;
            
        case MODE_RPM_DYNAMIC:
            my_printf("[SYSTEM] RPM_DYNAMIC mode - waiting for CAN 0x003\n");
            my_printf("[SYSTEM] LED: controlled by CAN activity\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            my_printf("[SYSTEM] FIXED_FREQUENCY mode\n");
            if(g_system_state.target_frequency_hz == 0) {
                my_printf("[WARNING] Frequency not set!\n");
            } else {
                my_printf("[SYSTEM] Frequency: %lu Hz\n", 
                         g_system_state.target_frequency_hz);
            }
            my_printf("[SYSTEM] LED: 1 Hz blink\n");
            break;
            
        case MODE_HI_IMPEDANCE:
            my_printf("[SYSTEM] HI_IMPEDANCE mode\n");
            my_printf("[SYSTEM] GPIO: INPUT (Hi-Z)\n");
            my_printf("[SYSTEM] SSR: ACTIVE (external signal)\n");
            break;
            
        case MODE_DISABLED:
            my_printf("[SYSTEM] DISABLED mode\n");
            my_printf("[SYSTEM] All timers stopped\n");
            my_printf("[SYSTEM] LED: OFF\n");
            break;
            
        case MODE_ERROR:
            my_printf("[SYSTEM] ERROR mode\n");
            break;
            
        default:
            break;
    }
    #endif
    
    // Реальные действия при входе
    switch(new_mode) {
        case MODE_RPM_DYNAMIC:
            g_system_state.rpm_mode_active = 0;  // Флаг будет установлен при первом сообщении
            break;
            
        case MODE_DISABLED:
            TIM1->CR1 &= ~TIM_CR1_CEN;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            TIM4->CR1 &= ~TIM_CR1_CEN;
            break;
            
        default:
            break;
    }
    
    // Инициализируем LED для новго режима
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    
    #if DEBUG_SYSTEM
    my_printf("[SYSTEM] Mode switch complete\n\n");
    #endif
}

// ============================================
// ПОЛУЧЕНИЕ ИМЕНИ РЕЖИМА
// ============================================

const char* get_mode_name(operation_mode_t mode)
{
    switch(mode) {
        case MODE_BOOT: return "BOOT";
        case MODE_RPM_DYNAMIC: return "RPM_DYNAMIC";
        case MODE_FIXED_FREQUENCY: return "FIXED_FREQ";
        case MODE_DISABLED: return "DISABLED";
        case MODE_HI_IMPEDANCE: return "HI_IMPEDANCE";
        case MODE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// ============================================
// ВРЕМЯ РАБОТЫ СИСТЕМЫ
// ============================================

uint32_t system_get_uptime_seconds(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t uptime_ms = current_time - g_system_state.boot_time;
    return uptime_ms / 1000;
}

// ============================================
// ПЕЧАТЬ СТАТУСА
// ============================================

void system_print_status(void)
{
    #if DEBUG_SYSTEM
    my_printf("\n=== SYSTEM STATUS ===\n");
    my_printf("Mode: %s\n", get_mode_name(g_system_state.current_mode));
    my_printf("Channels: 0x%02X\n", g_system_state.channel_mask);
    
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        my_printf("Target freq: %lu Hz (%.1f mHz)\n", 
                 g_system_state.target_frequency_hz,
                 g_system_state.target_frequency_mhz / 1000.0f);
    }
    
    if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        my_printf("RPM active: %s\n", g_system_state.rpm_mode_active ? "YES" : "NO");
    }
    
    my_printf("Hi-Z mode: %s\n", g_system_state.hi_impedance_active ? "ACTIVE" : "INACTIVE");
    my_printf("Last CAN: %lu ms ago\n", 
             HAL_GetTick() - g_system_state.last_can_command_time);
    my_printf("Uptime: %lu seconds\n", system_get_uptime_seconds());
    my_printf("===================\n\n");
    #endif
}

// ============================================
// УПРАВЛЕНИЕ КАНАЛАМИ
// ============================================

void set_channel_active(uint8_t channel_num, uint8_t active)
{
    if(channel_num >= 4) return;
    
    if(active) {
        g_system_state.channel_mask |= (1 << channel_num);
    } else {
        g_system_state.channel_mask &= ~(1 << channel_num);
    }
}

uint8_t is_channel_active(uint8_t channel_num)
{
    if(channel_num >= 4) return 0;
    return (g_system_state.channel_mask >> channel_num) & 0x01;
}

void set_all_channels_active(uint8_t active)
{
    g_system_state.channel_mask = active ? 0x0F : 0x00;
    
    #if DEBUG_SYSTEM
    my_printf("[SYSTEM] All channels: %s\n", active ? "ACTIVE" : "INACTIVE");
    #endif
}
