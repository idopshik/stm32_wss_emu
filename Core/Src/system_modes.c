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
#ifdef DEBUG_SYSTEM
    my_printf("[SYSTEM] Initializing modes...\n");
#endif

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
    g_system_state.led_boot_start_time = HAL_GetTick();
    
    // Подготовка к Hi-Z после boot
    g_system_state.pending_hi_z = 1;
    
#ifdef DEBUG_SYSTEM
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
#ifdef DEBUG_SYSTEM
        my_printf("[SYSTEM] Already in mode: %s\n", get_mode_name(new_mode));
#endif
        return;
    }
    
#ifdef DEBUG_SYSTEM
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
#ifdef DEBUG_SYSTEM
            my_printf("[SYSTEM] RPM mode ended\n");
#endif
            break;
            
        case MODE_FIXED_FREQUENCY:
#ifdef DEBUG_SYSTEM
            my_printf("[SYSTEM] Exiting FIXED_FREQUENCY\n");
#endif
            break;
            
        case MODE_HI_IMPEDANCE:
#ifdef DEBUG_SYSTEM
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
    
#ifdef DEBUG_SYSTEM
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
            
            TIM1->CR1 |= TIM_CR1_CEN;      // Включить таймер
            TIM1->DIER |= TIM_DIER_UIE;    // Включить прерывания
            TIM1->EGR = TIM_EGR_UG;        // Обнулить счетчик

            TIM3->CR1 |= TIM_CR1_CEN;      // Включить таймер
            TIM3->DIER |= TIM_DIER_UIE;    // Включить прерывания
            TIM3->EGR = TIM_EGR_UG;        // Обнулить счетчик

            TIM4->CR1 |= TIM_CR1_CEN;      // Включить таймер
            TIM4->DIER |= TIM_DIER_UIE;    // Включить прерывания
            TIM4->EGR = TIM_EGR_UG;        // Обнулить счетчик
            
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
    
#ifdef DEBUG_SYSTEM
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
#ifdef DEBUG_SYSTEM
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
    
#ifdef DEBUG_SYSTEM
    my_printf("[SYSTEM] All channels: %s\n", active ? "ACTIVE" : "INACTIVE");
#endif
}




// ============================================
// HI-Z РЕЖИМ - ВХОД
// ============================================

void enter_hi_impedance_mode(void)
{
#ifdef DEBUG_SYSTEM
    my_printf("[HI-Z] Entering Hi-Z mode\n");
#endif

    // 1. Отключение всех таймеров
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;  // ЕДИНСТВЕННЫЙ PWM таймер!
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // 2. Отключение прерываний
    TIM1->DIER = 0;
    TIM2->DIER = 0;
    TIM3->DIER = 0;
    TIM4->DIER = 0;
    
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_DisableIRQ(TIM3_IRQn);
    NVIC_DisableIRQ(TIM4_IRQn);
    
    // 3. Отключение тактирования
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM2_CLK_DISABLE();  // TIM2 тоже отключаем!
    __HAL_RCC_TIM3_CLK_DISABLE();
    __HAL_RCC_TIM4_CLK_DISABLE();

    // 4. ВСЕ GPIO КОЛЁС В INPUT MODE
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Pull-down по умолчанию
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // Все пины колёс:
    // PA8 - FL wheel (GPIO)
    // PA15 - FR wheel (TIM2_CH1) - ОСОБО ВАЖНЫЙ!
    // PB9 - RL wheel (GPIO)
    // PA6 - RR wheel (GPIO)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 5. Сброс альтернативных функций
    // PA15 (TIM2_CH1) - сброс AF!
    GPIOA->AFR[1] &= ~(0xF << 28);  // PA15: биты 28-31
    
    // Другие пины (на всякий случай):
    GPIOA->AFR[0] &= ~((0xF << 24) | (0xF << 0));  // PA6, PA8
    GPIOB->AFR[1] &= ~(0xF << 4);                  // PB9

    // 6. Включение SSR (PB10)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

    // 7. Обновление состояния
    g_system_state.hi_impedance_active = 1;
    system_switch_mode(MODE_HI_IMPEDANCE);

#ifdef DEBUG_SYSTEM
    my_printf("[HI-Z] All wheels INPUT, PA15(TIM2) AF cleared\n");
#endif
}


// ============================================
// HI-Z РЕЖИМ - ВЫХОД
// ============================================

void exit_hi_impedance_mode(void)
{
    my_printf("[HI-Z EXIT] 1. Enabling clocks\n");
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    my_printf("[HI-Z EXIT] 2. Clocks enabled\n");

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    my_printf("[HI-Z EXIT] 3. SSR disabled\n");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    my_printf("[HI-Z EXIT] 9. Timer config start\n");
    TIM2->CR1 = 0;
    TIM2->PSC = 12;
    TIM2->ARR = 65535;
    TIM2->CNT = 0;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CCR1 = 32767;
    
    TIM1->CR1 = 0;
    TIM1->PSC = 24;
    TIM1->ARR = 65535;
    TIM1->CNT = 0;
    
    TIM3->CR1 = 0;
    TIM3->PSC = 24;
    TIM3->ARR = 65535;
    TIM3->CNT = 0;
    
    TIM4->CR1 = 0;
    TIM4->PSC = 24;
    TIM4->ARR = 65535;
    TIM4->CNT = 0;

    my_printf("[HI-Z EXIT] 12. DIER setup\n");
    TIM1->DIER |= TIM_DIER_UIE;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM4->DIER |= TIM_DIER_UIE;

    my_printf("[HI-Z EXIT] 14. NVIC enable SKIPPED (will do in main loop)\n");
    // ← НЕ ВКЛЮЧАЕМ ПРЕРЫВАНИЯ ЗДЕСЬ!
    // Вместо этого устанавливаем флаг

    g_system_state.hi_impedance_active = 0;
    
    my_printf("[HI-Z EXIT] COMPLETE\n");
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
