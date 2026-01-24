/**
 * system_modes.c - ОБНОВЛЁННАЯ ВЕРСИЯ
 * 
 * Добавлено:
 * - Вызов analog_follower_start/stop при переключении режимов
 * - Улучшенная обработка переходов
 */

#include "system_modes.h"
#include "analog_follower.h"
#include <stdio.h>

extern void my_printf(const char *fmt, ...);

system_state_t g_system_state;

const char* mode_names[] = {
    [MODE_BOOT] = "BOOT",
    [MODE_RPM_DYNAMIC] = "RPM_DYNAMIC",
    [MODE_FIXED_FREQUENCY] = "FIXED_FREQ",
    [MODE_PWM] = "PWM",
    [MODE_ANALOG_FOLLOW] = "ANALOG_FOLLOW",
    [MODE_DISABLED] = "DISABLED",
    [MODE_ERROR] = "ERROR"
};

void system_init_modes(void)
{
    my_printf("[SYSTEM] Initializing modes...\n");
    
    g_system_state.current_mode = MODE_BOOT;
    g_system_state.channel_mask = 0x0F;
    g_system_state.target_frequency_hz = 0;
    g_system_state.pwm_duty_percent = 50;
    
    for(int i = 0; i < 4; i++) {
        g_system_state.psc_values[i] = 24;
        g_system_state.arr_values[i] = 59999;
    }
    
    g_system_state.analog_signal_present = 0;
    g_system_state.last_can_command_time = HAL_GetTick();
    g_system_state.led_last_toggle_time = 0;
    g_system_state.led_state = 0;
    
    my_printf("[SYSTEM] Initialized. Mode: %s\n", 
             get_mode_name(g_system_state.current_mode));
}

void system_switch_mode(operation_mode_t new_mode)
{
    operation_mode_t old_mode = g_system_state.current_mode;
    
    if(old_mode == new_mode) {
        my_printf("[SYSTEM] Already in mode: %s\n", get_mode_name(new_mode));
        return;
    }
    
    my_printf("\n[SYSTEM] Mode transition: %s -> %s\n", 
              get_mode_name(old_mode), 
              get_mode_name(new_mode));
    
    // ============================================
    // ДЕЙСТВИЯ ПРИ ВЫХОДЕ ИЗ СТАРОГО РЕЖИМА
    // ============================================
    
    switch(old_mode) {
        case MODE_ANALOG_FOLLOW:
            // КРИТИЧНО: останавливаем analog follower
            my_printf("[SYSTEM] Stopping analog follower...\n");
            analog_follower_stop();
            break;
            
        case MODE_FIXED_FREQUENCY:
            // Частота уже установлена - оставляем как есть
            my_printf("[SYSTEM] Exiting fixed frequency mode\n");
            break;
            
        case MODE_RPM_DYNAMIC:
            my_printf("[SYSTEM] Exiting RPM mode\n");
            break;
            
        default:
            break;
    }
    
    // Устанавливаем новый режим
    g_system_state.current_mode = new_mode;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    // ============================================
    // ДЕЙСТВИЯ ПРИ ВХОДЕ В НОВЫЙ РЕЖИМ
    // ============================================
    
    switch(new_mode) {
        case MODE_BOOT:
            my_printf("[SYSTEM] BOOT mode\n");
            break;
            
        case MODE_RPM_DYNAMIC:
            my_printf("[SYSTEM] RPM mode - waiting for CAN 0x003\n");
            my_printf("[SYSTEM] LED: controlled by CAN activity (TIM8)\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            my_printf("[SYSTEM] FIXED FREQUENCY mode\n");
            if(g_system_state.target_frequency_hz == 0) {
                my_printf("[WARNING] Frequency not set!\n");
            } else {
                my_printf("[SYSTEM] Frequency: %lu Hz\n", 
                         g_system_state.target_frequency_hz);
            }
            my_printf("[SYSTEM] LED: 1 Hz blink\n");
            break;
            
        case MODE_PWM:
            my_printf("[SYSTEM] PWM mode\n");
            my_printf("[SYSTEM] Freq: %lu Hz, Duty: %d%%\n",
                     g_system_state.target_frequency_hz,
                     g_system_state.pwm_duty_percent);
            my_printf("[SYSTEM] LED: 0.5 Hz blink\n");
            break;
            
        case MODE_ANALOG_FOLLOW:
            // КРИТИЧНО: запускаем analog follower
            my_printf("[SYSTEM] ANALOG FOLLOW mode\n");
            my_printf("[SYSTEM] LED: 2 Hz when signal / ON when no signal\n");
            analog_follower_start();
            break;
            
        case MODE_DISABLED:
            my_printf("[SYSTEM] DISABLED mode\n");
            my_printf("[SYSTEM] Stopping all timers...\n");
            
            // Останавливаем все таймеры
            TIM1->CR1 &= ~TIM_CR1_CEN;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            TIM4->CR1 &= ~TIM_CR1_CEN;
            
            my_printf("[SYSTEM] LED: OFF\n");
            break;
            
        case MODE_ERROR:
            my_printf("[SYSTEM] ERROR mode\n");
            my_printf("[SYSTEM] LED: 10 Hz fast blink\n");
            break;
    }
    
    // Сбрасываем LED
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
    
    my_printf("[SYSTEM] Mode switch complete\n\n");
}

const char* get_mode_name(operation_mode_t mode)
{
    if(mode <= MODE_DISABLED) {
        return mode_names[mode];
    }
    return "UNKNOWN";
}

void system_print_status(void)
{
    my_printf("\n=== SYSTEM STATUS ===\n");
    my_printf("Mode: %s\n", get_mode_name(g_system_state.current_mode));
    my_printf("Channels: ");
    for(int i = 0; i < 4; i++) {
        my_printf("%c", is_channel_active(i) ? '1' : '0');
    }
    my_printf(" (0x%02X)\n", g_system_state.channel_mask);
    
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY || 
       g_system_state.current_mode == MODE_PWM) {
        my_printf("Target freq: %lu Hz\n", g_system_state.target_frequency_hz);
    }
    
    if(g_system_state.current_mode == MODE_PWM) {
        my_printf("PWM duty: %d%%\n", g_system_state.pwm_duty_percent);
    }
    
    if(g_system_state.current_mode == MODE_ANALOG_FOLLOW) {
        my_printf("Analog signal: %s\n", 
                 g_system_state.analog_signal_present ? "PRESENT" : "ABSENT");
        my_printf("Frequency: %lu Hz\n", analog_follower_get_frequency());
    }
    
    my_printf("Last CAN: %lu ms ago\n", 
             HAL_GetTick() - g_system_state.last_can_command_time);
    my_printf("===================\n\n");
}

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
    my_printf("[SYSTEM] All channels: %s\n", active ? "ACTIVE" : "INACTIVE");
}
