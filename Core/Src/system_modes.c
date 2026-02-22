/**
 * system_modes.c - VERSION 4.5 (DDS INTEGRATION)
 * 
 * Изменения v4.5:
 * - RPM режим использует DDS (wss_dds.c) вместо таймеров
 * - GPIO переключаются между DDS mode (Output) и Timer mode (AF/Output)
 * - FIXED режим продолжает использовать таймеры с seamless switching
 */

#include <stdio.h>
#include "main.h"
#include "stm32g4xx_hal.h"
#include "system_modes.h"
#include "wss_dds.h"  // ← DDS для RPM режима

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

system_state_t g_system_state;

static uint8_t dds_initialized = 0;  // Флаг инициализации DDS

/**
 * Настроить GPIO для DDS режима (RPM)
 * Все 4 пина на GPIOA в OUTPUT_OD для DMA→BSRR
 */
static void configure_gpio_for_dds(void)
{
    printf("[SYSTEM] Configuring GPIO for DDS (RPM mode)...\n");
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Все 4 пина: PA8 (FL), PA15 (FR), PA0 (RL), PA6 (RR)
    // Open-drain для совместимости с 5V внешним pull-up
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    printf("[SYSTEM] GPIO configured: PA8/PA15/PA0/PA6 as OUTPUT_OD for DDS\n");
}

/**
 * Настроить GPIO для FIXED режима (таймеры)
 * PA15 - AF для TIM2 (hardware toggle)
 * PA8/PA0/PA6 - Output для manual toggle в прерываниях
 */
static void configure_gpio_for_timers(void)
{
    printf("[SYSTEM] Configuring GPIO for Timers (FIXED mode)...\n");
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // TIM2_CH1 (PA15, FR) - Alternate Function Open-Drain для hardware toggle
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM1 (PA8, FL), TIM3 (PA0, RL), TIM4 (PA6, RR) - manual toggle в прерываниях
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_0 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    printf("[SYSTEM] GPIO configured: PA15=AF_OD (TIM2), PA8/PA0/PA6=OUTPUT_PP (manual)\n");
}

/**
 * Восстанавливает GPIO после EXTERNAL_SIGNAL режима
 */
static void restore_gpio_after_external(void)
{
    printf("[SYSTEM] Restoring GPIO after EXTERNAL mode...\n");
    
    // После EXTERNAL режима возвращаемся в то состояние GPIO,
    // которое требуется для текущего режима
    if (g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        configure_gpio_for_dds();
    } else if (g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        configure_gpio_for_timers();
    }
}

void system_init_modes(void)
{
    printf("[SYSTEM] Initializing...\n");

    g_system_state.current_mode = MODE_RPM_DYNAMIC;
    g_system_state.channel_mask = 0x0F;  // Все каналы по умолчанию
    g_system_state.target_frequency_hz = 0;
    g_system_state.target_frequency_mhz = 0;
    
    // Инициализация значений таймеров (для FIXED режима)
    for(int i = 0; i < 4; i++) {
        if(i == 1) {  // TIM2 (FR) - 32-битный
            g_system_state.psc_values[i] = 12;
            g_system_state.arr_values_32bit[i] = 0xFFFFFFFF;
        } else {      // Остальные - 16-битные
            g_system_state.psc_values[i] = 24;
            g_system_state.arr_values[i] = 59999;
        }
    }
    
    g_system_state.last_can_command_time = HAL_GetTick();
    g_system_state.boot_time = HAL_GetTick();
    g_system_state.led_last_toggle_time = HAL_GetTick();
    g_system_state.led_state = 0;
    g_system_state.rpm_signal_active = 0;
    
    g_system_state.led_boot_flashed = 0;
    g_system_state.led_boot_start_time = HAL_GetTick();
    
    // SSR выключен при загрузке
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // PB10 = LOW
    
    // Инициализация DDS для RPM режима
    configure_gpio_for_dds();
    wss_dds_init();
    dds_initialized = 1;
    
    printf("[SYSTEM] Started in RPM mode with DDS.\n");
    printf("[SYSTEM] LED will turn ON when RPM data arrives.\n");
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
    
    // Остановить таймеры (используются в FIXED режиме)
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    TIM1->CCER = 0;
    TIM2->CCER = 0;
    TIM3->CCER = 0;
    TIM4->CCER = 0;
    
    // DDS продолжает работать в фоне, но wss_dds_set_rpm с нулями
    // остановит переключения (step=0 → пины замораживаются)
    
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
            // В BOOT режиме и DDS и таймеры остановлены
            break;
        
        case MODE_RPM_DYNAMIC:
            printf("[SYSTEM] RPM MODE: DDS active, waiting for CAN 0x003 signals\n");
            printf("[SYSTEM] LED: ON when signal arrives, OFF after 500ms silence\n");
            printf("[SYSTEM] Active channels: 0x%02X\n", g_system_state.channel_mask);
            
            // Переключить GPIO для DDS
            configure_gpio_for_dds();
            
            // DDS уже инициализирован и работает
            // Просто установим нулевые скорости (пины замораживаются в LOW)
            uint16_t zero_speeds[4] = {0, 0, 0, 0};
            wss_dds_set_rpm(zero_speeds);
            
            printf("[SYSTEM] DDS ready. Waiting for CAN data.\n");
            break;
        
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] FIXED MODE: Frequency = %lu Hz (timer)\n", 
                   g_system_state.target_frequency_hz);
            printf("[SYSTEM] Output frequency = %lu Hz\n",
                   g_system_state.target_frequency_hz / 2);
            printf("[SYSTEM] LED: Constant blink 500ms\n");
            
            // Переключить GPIO для таймеров
            configure_gpio_for_timers();
            
            // Остановить DDS генерацию (step=0)
            uint16_t stop_dds[4] = {0, 0, 0, 0};
            wss_dds_set_rpm(stop_dds);
            
            // Запустить таймеры согласно маске
            if(g_system_state.channel_mask == 0x02) {
                // ONLY_FR: только TIM2
                printf("[SYSTEM] ONLY_FR: TIM2 enabled\n");
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM2->CCER |= TIM_CCER_CC1E;
                
            } else if(g_system_state.channel_mask == 0x0F) {
                // ALL_FOUR: все четыре таймера
                printf("[SYSTEM] ALL_FOUR: all timers enabled\n");
                
                // TIM1 (PA8, FL) - manual toggle
                TIM1->CR1 |= TIM_CR1_CEN;
                
                // TIM2 (PA15, FR) - hardware toggle
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM2->CCER |= TIM_CCER_CC1E;
                
                // TIM3 (PA0, RL) - manual toggle
                TIM3->CR1 |= TIM_CR1_CEN;
                
                // TIM4 (PA6, RR) - manual toggle
                TIM4->CR1 |= TIM_CR1_CEN;
                
            } else {
                printf("[SYSTEM WARNING] Unexpected channel_mask: 0x%02X\n",
                       g_system_state.channel_mask);
            }
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            printf("[SYSTEM] EXTERNAL MODE: GPIO in HIGH-Z, SSR ON for optocoupler\n");
            printf("[SYSTEM] LED: Slow blink 2500ms\n");
            
            // Остановить DDS
            uint16_t stop_ext[4] = {0, 0, 0, 0};
            wss_dds_set_rpm(stop_ext);
            
            // Переводим GPIO в INPUT (HIGH-Z)
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            
            GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_6;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            // Включить SSR для питания оптопары
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

void update_system_indicators(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Дебаунс: обновляем не чаще 10ms
    if(current_time - last_update < 10) {
        return;
    }
    last_update = current_time;
    
    switch(g_system_state.current_mode) {
        
        case MODE_BOOT:
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
            if(current_time - g_system_state.led_last_toggle_time > 500) {
                g_system_state.rpm_signal_active = 0;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // OFF
            }
            else if(g_system_state.rpm_signal_active) {
                static uint32_t last_blink = 0;
                if(current_time - last_blink >= 100) {
                    last_blink = current_time;
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
                }
            }
            break;
        
        case MODE_FIXED_FREQUENCY:
            if(current_time - g_system_state.led_last_toggle_time >= 500) {
                g_system_state.led_last_toggle_time = current_time;
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
            }
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            if(current_time - g_system_state.led_last_toggle_time >= 2500) {
                g_system_state.led_last_toggle_time = current_time;
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
            }
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
        printf("[SYSTEM] Channel %d ENABLED (mask: 0x%02X)\n", 
               channel_num, g_system_state.channel_mask);
    } else {
        g_system_state.channel_mask &= ~(1 << channel_num);
        printf("[SYSTEM] Channel %d DISABLED (mask: 0x%02X)\n", 
               channel_num, g_system_state.channel_mask);
    }
}

uint8_t is_channel_enabled_in_current_mode(uint8_t channel_num)
{
    if(channel_num >= 4) return 0;
    
    if(g_system_state.current_mode == MODE_BOOT) {
        return 0;
    }
    
    if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        return (g_system_state.channel_mask >> channel_num) & 0x01;
    }
    
    return 1;
}
