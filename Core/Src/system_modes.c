/**
 * system_modes.c - VERSION 3
 * 
 * Изменения:
 * - Удалены все аналоговые режимы и функции
 * - Упрощена логика режимов (только RPM, FIXED, HI-Z, DISABLED)
 * - Добавлен флаг rpm_mode_active для LED логики
 */

#include <stdio.h>
#include "main.h"
#include "stm32g4xx_hal.h"
#include "system_modes.h"

extern void printf(const char *fmt, ...);

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
    printf("[SYSTEM] Initializing modes...\n");

    g_system_state.current_mode = MODE_BOOT;
    g_system_state.channel_mask = 0x0F;
    g_system_state.target_frequency_hz = 0;
    g_system_state.target_frequency_mhz = 0;
    g_system_state.pwm_duty_percent = 50;
    
    for(int i = 0; i < 4; i++) {
        if(i == 1) {  // TIM2 (FR) - 32-битный
            g_system_state.psc_values[i] = 12;        // PSC для 32-бит
            g_system_state.arr_values[i] = 0xFFFF;    // Макс для 16-бит части
            g_system_state.arr_values_32bit[i] = 0xFFFFFFFF;  // Полный 32-бит
        } else {  // TIM1, TIM3, TIM4 - 16-битные
            g_system_state.psc_values[i] = 24;
            g_system_state.arr_values[i] = 59999;
            g_system_state.arr_values_32bit[i] = 59999;
        }
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
    g_system_state.led_boot_start_time = HAL_GetTick();
    
    // Подготовка к Hi-Z после boot
    g_system_state.pending_hi_z = 1;
    
    printf("[SYSTEM] Initialized. Mode: %s\n", 
             get_mode_name(g_system_state.current_mode));
}

// ============================================
// ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ
// ============================================

void system_switch_mode(operation_mode_t new_mode)
{
    operation_mode_t old_mode = g_system_state.current_mode;
    
    if(old_mode == new_mode) {
        printf("[SYSTEM] Already in mode: %s\n", get_mode_name(new_mode));
        return;
    }
    
    printf("\n[SYSTEM] Mode transition: %s → %s\n", get_mode_name(old_mode), get_mode_name(new_mode));
    
    // ============================================
    // ВЫХОД ИЗ СТАРОГО РЕЖИМА
    // ============================================
    
    // Специфичные действия при выходе
    switch(old_mode) {
        case MODE_RPM_DYNAMIC:
            g_system_state.rpm_mode_active = 0;
            printf("[SYSTEM] RPM mode ended\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] Exiting FIXED_FREQUENCY\n");
            break;
            
        case MODE_HI_IMPEDANCE:
            printf("[SYSTEM] Exiting HI_IMPEDANCE\n");
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
    
    switch(new_mode) {
        case MODE_BOOT:
            printf("[SYSTEM] BOOT mode\n");
            break;
            
        case MODE_RPM_DYNAMIC:
            printf("[SYSTEM] RPM_DYNAMIC mode - waiting for CAN 0x003\n");
            printf("[SYSTEM] LED: controlled by CAN activity\n");
            break;
            
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] FIXED_FREQUENCY mode\n");
            if(g_system_state.target_frequency_hz == 0) {
                printf("[WARNING] Frequency not set!\n");
            } else {
                printf("[SYSTEM] Frequency: %lu Hz\n", 
                         g_system_state.target_frequency_hz);
            }
            printf("[SYSTEM] LED: 1 Hz blink\n");
            break;
            
        case MODE_HI_IMPEDANCE:
            printf("[SYSTEM] HI_IMPEDANCE mode\n");
            printf("[SYSTEM] GPIO: INPUT (Hi-Z)\n");
            printf("[SYSTEM] SSR: ACTIVE (external signal)\n");
            break;
            
        case MODE_DISABLED:
            printf("[SYSTEM] DISABLED mode\n");
            printf("[SYSTEM] All timers stopped\n");
            printf("[SYSTEM] LED: OFF\n");
            break;
            
        case MODE_ERROR:
            printf("[SYSTEM] ERROR mode\n");
            break;
            
        default:
            break;
    }
    
    // Реальные действия при входе
    switch(new_mode) {
        case MODE_RPM_DYNAMIC:
            g_system_state.rpm_mode_active = 0;  // Флаг будет установлен при первом сообщении
            
            /* TIM1->CR1 |= TIM_CR1_CEN;      // Включить таймер */
            /* TIM1->DIER |= TIM_DIER_UIE;    // Включить прерывания */
            /* TIM1->EGR = TIM_EGR_UG;        // Обнулить счетчик */

            /* TIM3->CR1 |= TIM_CR1_CEN;      // Включить таймер */
            /* TIM3->DIER |= TIM_DIER_UIE;    // Включить прерывания */
            /* TIM3->EGR = TIM_EGR_UG;        // Обнулить счетчик */

            /* TIM4->CR1 |= TIM_CR1_CEN;      // Включить таймер */
            /* TIM4->DIER |= TIM_DIER_UIE;    // Включить прерывания */
            /* TIM4->EGR = TIM_EGR_UG;        // Обнулить счетчик */
            
            break;
            
        case MODE_DISABLED:
            /* TIM1->CR1 &= ~TIM_CR1_CEN; */
            /* TIM2->CR1 &= ~TIM_CR1_CEN; */
            /* TIM3->CR1 &= ~TIM_CR1_CEN; */
            /* TIM4->CR1 &= ~TIM_CR1_CEN; */
            break;
            
        default:
            break;
    }
    
    // Инициализируем LED для нового режима
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    
    printf("[SYSTEM] Mode switch complete\n\n");
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
    printf("\n=== SYSTEM STATUS ===\n");
    printf("Mode: %s\n", get_mode_name(g_system_state.current_mode));
    printf("Channels: 0x%02X\n", g_system_state.channel_mask);
    
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        printf("Target freq: %lu Hz (%.1f mHz)\n", 
                 g_system_state.target_frequency_hz,
                 g_system_state.target_frequency_mhz / 1000.0f);
    }
    
    if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        printf("RPM active: %s\n", g_system_state.rpm_mode_active ? "YES" : "NO");
    }
    
    printf("Hi-Z mode: %s\n", g_system_state.hi_impedance_active ? "ACTIVE" : "INACTIVE");
    printf("Last CAN: %lu ms ago\n", 
             HAL_GetTick() - g_system_state.last_can_command_time);
    printf("Uptime: %lu seconds\n", system_get_uptime_seconds());
    printf("===================\n\n");
}

// ============================================
// УПРАВЛЕНИЕ КАНАЛАМИ
// ============================================

void set_channel_active(uint8_t channel_num, uint8_t active)
{
    printf("set_chan_active_called");
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
    
    printf("[SYSTEM] All channels: %s\n", active ? "ACTIVE" : "INACTIVE");
}




// ============================================
// HI-Z РЕЖИМ - ВХОД
// ============================================

void enter_hi_impedance_mode(void)
{
    printf("[HI-Z] Entering TRUE Hi-Z mode\n");
    
    // 1. Остановить все таймеры и ОТКЛЮЧИТЬ их выходы
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER = 0;  // Отключаем ВСЕ каналы TIM1
    
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CCER &= ~TIM_CCER_CC1E;  // ТОЛЬКО канал 1 (PA15)
    
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->CCER = 0;
    
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CCER = 0;
    
    // 2. НЕ ОТКЛЮЧАТЬ тактирование! Таймеры должны сохранять настройки
    
    // 3. Настроить ВСЕ пины как ANALOG (настоящий Hi-Z)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;      // ← АНАЛОГОВЫЙ!
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // ← БЕЗ подтяжки!
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // ВСЕ пины колес:
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 4. Сбросить альтернативные функции (очистить AFR)
    GPIOA->AFR[0] = 0;  // PA0-PA7
    GPIOA->AFR[1] = 0;  // PA8-PA15
    GPIOB->AFR[0] = 0;  // PB0-PB7
    GPIOB->AFR[1] = 0;  // PB8-PB15
    
    // 5. Включить SSR (PB10)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    
    // 6. Обновить состояние
    g_system_state.hi_impedance_active = 1;
    
    printf("[HI-Z] All pins in TRUE Hi-Z (analog mode)\n");
    printf("[HI-Z] SSR ON, timers frozen but configured\n");
}


// ============================================
// HI-Z РЕЖИМ - ВЫХОД
// ============================================

void exit_hi_impedance_mode(void)
{
    printf("[HI-Z EXIT] Restoring outputs\n");
    
    // 1. Выключить SSR
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    
    // 2. Восстановить PA15 как TIM2_CH1
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // Альтернативная функция
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;   // TIM2_CH1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. Остальные пины как обычные выходы (или что нужно)
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 4. Включить выход TIM2 (но НЕ запускать счет!)
    TIM2->CCER |= TIM_CCER_CC1E;  // Канал 1 активен
    
    // 5. Таймеры остаются ОСТАНОВЛЕННЫМИ (CR1.CEN = 0)
    // Их запустит system_switch_mode() когда нужно
    
    g_system_state.hi_impedance_active = 0;
    
    printf("[HI-Z EXIT] Outputs restored, timers ready\n");
}



static void led_blink_pattern(uint32_t current_time, uint32_t interval_ms)
{
    if(current_time - g_system_state.led_last_toggle_time >= interval_ms) {
        g_system_state.led_state = !g_system_state.led_state;
        g_system_state.led_last_toggle_time = current_time;
        
        if(g_system_state.led_state) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // EXT_LED ON
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);    // EXT_LED OFF
        }
    }
}

void update_system_indicators(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(current_time - last_update < 10) {
        return;
    }
    last_update = current_time;
    
    switch(g_system_state.current_mode) {
        case MODE_BOOT:
            if(!g_system_state.led_boot_flashed) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                g_system_state.led_boot_start_time = current_time;
                g_system_state.led_boot_flashed = 1;
            }
            
            if(current_time - g_system_state.led_boot_start_time >= 1500) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            }
            break;
            
        case MODE_RPM_DYNAMIC:
            if(g_system_state.rpm_mode_active) {
                led_blink_pattern(current_time, 100);  // Мигает если приём данных
            } else {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // OFF если нет данных
            }
            break;
            
        case MODE_FIXED_FREQUENCY:
            led_blink_pattern(current_time, 500);
            break;
            
        case MODE_HI_IMPEDANCE:
            led_blink_pattern(current_time, 2500);
            break;
            
        case MODE_DISABLED:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            break;
            
        case MODE_ERROR:
            led_blink_pattern(current_time, 50);
            break;
            
        default:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            break;
    }
}
