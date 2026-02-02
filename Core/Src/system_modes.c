/**
 * system_modes.c - VERSION 4.2 (ONLY 3 MODES + SSR FOR EXTERNAL)
 * 
 * Переделано с нуля:
 * - Только 3 режима: RPM, FIXED, EXTERNAL
 * - Убран DISABLED и Hi-Z (переименован в EXTERNAL)
 * - Убран pending_hi_z и автоматические переходы
 * - Система ТОЛЬКО по CAN командам
 * - LED просто: BOOT вспышка, потом по режимам
 * - SSR (PB10) включен в EXTERNAL режиме для питания оптопары
 */

#include <stdio.h>
#include "main.h"
#include "stm32g4xx_hal.h"
#include "system_modes.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

system_state_t g_system_state;

void system_init_modes(void)
{
    printf("[SYSTEM] Initializing...\n");

    // BOOT состояние: система ждёт CAN команды
    g_system_state.current_mode = MODE_BOOT;
    g_system_state.channel_mask = 0x0F;
    g_system_state.target_frequency_hz = 0;
    g_system_state.target_frequency_mhz = 0;
    
    for(int i = 0; i < 4; i++) {
        if(i == 1) {
            g_system_state.psc_values[i] = 12;
            g_system_state.arr_values_32bit[i] = 0xFFFFFFFF;
        } else {
            g_system_state.psc_values[i] = 24;
            g_system_state.arr_values[i] = 59999;
        }
    }
    
    g_system_state.last_can_command_time = HAL_GetTick();
    g_system_state.boot_time = HAL_GetTick();
    g_system_state.led_last_toggle_time = 0;
    g_system_state.led_state = 0;
    g_system_state.rpm_signal_active = 0;
    
    // BOOT LED: одна вспышка 1500ms
    g_system_state.led_boot_flashed = 0;
    g_system_state.led_boot_start_time = HAL_GetTick();
    
    // Убедиться, что SSR выключен при загрузке
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // PB10 = LOW
    
    printf("[SYSTEM] Ready. Waiting for CAN command.\n");
}

void system_switch_mode(operation_mode_t new_mode)
{
    operation_mode_t old_mode = g_system_state.current_mode;
    
    if(old_mode == new_mode && new_mode != MODE_BOOT) {
        printf("[SYSTEM] Already in mode: %s\n", get_mode_name(new_mode));
        return;
    }
    
    printf("\n[SYSTEM] Mode change: %s → %s\n", get_mode_name(old_mode), get_mode_name(new_mode));
    
    // ============ ВЫХОД ИЗ СТАРОГО РЕЖИМА ============
    
    // Остановить все таймеры
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // Отключить выходы
    TIM1->CCER = 0;
    TIM2->CCER = 0;
    TIM3->CCER = 0;
    TIM4->CCER = 0;
    
    // Выключить SSR при выходе из EXTERNAL режима
    if(old_mode == MODE_EXTERNAL_SIGNAL) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // PB10 = LOW
        printf("[SYSTEM] SSR OFF\n");
    }
    
    g_system_state.rpm_signal_active = 0;
    
    // ============ ПЕРЕКЛЮЧЕНИЕ ============
    
    g_system_state.current_mode = new_mode;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    // ============ ВХОД В НОВЫЙ РЕЖИМ ============
    
    switch(new_mode) {
        
        case MODE_BOOT:
            printf("[SYSTEM] BOOT: LED will flash for 1500ms\n");
            break;
        
        case MODE_RPM_DYNAMIC:
            printf("[SYSTEM] RPM MODE: Waiting for CAN 0x003 signals\n");
            printf("[SYSTEM] LED: ON when signal arrives, OFF after 500ms silence\n");
            
            // Запустить таймеры согласно маске
            if(g_system_state.channel_mask & 0x01) {
                TIM1->CR1 |= TIM_CR1_CEN;
                TIM1->DIER |= TIM_DIER_UIE;
            }
            if(g_system_state.channel_mask & 0x02) {
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM2->DIER |= TIM_DIER_UIE;
            }
            if(g_system_state.channel_mask & 0x04) {
                TIM3->CR1 |= TIM_CR1_CEN;
                TIM3->DIER |= TIM_DIER_UIE;
            }
            if(g_system_state.channel_mask & 0x08) {
                TIM4->CR1 |= TIM_CR1_CEN;
                TIM4->DIER |= TIM_DIER_UIE;
            }
            break;
        
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] FIXED MODE: Frequency = %lu Hz\n", g_system_state.target_frequency_hz);
            printf("[SYSTEM] LED: Constant blink 500ms\n");
            
            // Запустить все таймеры на фиксированной частоте
            TIM1->CR1 |= TIM_CR1_CEN;
            TIM2->CR1 |= TIM_CR1_CEN;
            TIM3->CR1 |= TIM_CR1_CEN;
            TIM4->CR1 |= TIM_CR1_CEN;
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            printf("[SYSTEM] EXTERNAL MODE: GPIO in HIGH-Z, SSR ON for optocoupler\n");
            printf("[SYSTEM] LED: Slow blink 2500ms\n");
            
            // Переводим GPIO в INPUT (HIGH-Z)
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            
            GPIO_InitStruct.Pin = GPIO_PIN_8;   // FL
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_15;  // FR
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_6;   // RL
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_9;   // RR
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            
            // Очистить AFR
            GPIOA->AFR[0] = 0;
            GPIOA->AFR[1] = 0;
            GPIOB->AFR[0] = 0;
            GPIOB->AFR[1] = 0;
            
            // ✅ ВКЛЮЧИТЬ SSR для питания оптопары
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // PB10 = HIGH
            printf("[SYSTEM] SSR ON: Optocoupler powered\n");
            break;
        
        default:
            break;
    }
    
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    
    printf("[SYSTEM] Mode change complete\n\n");
}

const char* get_mode_name(operation_mode_t mode)
{
    switch(mode) {
        case MODE_BOOT: return "BOOT";
        case MODE_RPM_DYNAMIC: return "RPM";
        case MODE_FIXED_FREQUENCY: return "FIXED";
        case MODE_EXTERNAL_SIGNAL: return "EXTERNAL";
        default: return "UNKNOWN";
    }
}

uint32_t system_get_uptime_seconds(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t uptime_ms = current_time - g_system_state.boot_time;
    return uptime_ms / 1000;
}

// ============================================
// LED ИНДИКАЦИЯ
// ============================================

static void led_blink_pattern(uint32_t current_time, uint32_t interval_ms)
{
    if(current_time - g_system_state.led_last_toggle_time >= interval_ms) {
        g_system_state.led_state = !g_system_state.led_state;
        g_system_state.led_last_toggle_time = current_time;
        
        if(g_system_state.led_state) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // LED ON
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);    // LED OFF
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
            // Одна вспышка 1500ms при загрузке
            if(!g_system_state.led_boot_flashed) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // ON
                g_system_state.led_boot_start_time = current_time;
                g_system_state.led_boot_flashed = 1;
            }
            
            if(current_time - g_system_state.led_boot_start_time >= 1500) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);    // OFF
            }
            break;
        
        case MODE_RPM_DYNAMIC:
            // Мигает ТОЛЬКО если есть сигнал
            // Гаснет если нет сигнала >500ms
            if(g_system_state.rpm_signal_active) {
                led_blink_pattern(current_time, 100);  // Мигание 100ms
            } else {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // OFF
            }
            break;
        
        case MODE_FIXED_FREQUENCY:
            // Постоянное мигание 500ms
            led_blink_pattern(current_time, 500);
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            // Медленное мигание 2500ms
            led_blink_pattern(current_time, 2500);
            break;
        
        default:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // OFF
            break;
    }
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
