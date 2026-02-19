/**
 * system_modes.c - VERSION 4.4 (CHANNEL MASK + PSC RESTORE)
 * 
 * Исправления v4.4:
 * - MODE_RPM_DYNAMIC: восстановление TIM2->PSC = 12 при входе в режим
 * - MODE_FIXED_FREQUENCY: учёт channel_mask (ONLY_FR vs ALL_FOUR)
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



/**
 * Восстанавливает GPIO как таймерные выходы после EXTERNAL_SIGNAL режима
 * PA15 восстанавливается в режиме Alternate Function Open-Drain
 * для совместимости с 5V внешним pull-up резистором
 */
static void restore_gpio_after_external(void)
{
    printf("[SYSTEM] Restoring GPIO...\n");
    
    
    // 2. Восстановить GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // TIM2_CH1 - Output Compare with Open-Drain (for 5V external pull-up)
    GPIO_InitStruct.Pin = GPIO_PIN_15;        // PA15 (FR channel)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;   // Alternate Function Open-Drain
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // TIM2 Channel 1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM1 - manual toggle in interrupt handler
    GPIO_InitStruct.Pin = GPIO_PIN_8;         // PA8
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Simple output
    GPIO_InitStruct.Alternate = 0;            // No AF
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM3 - manual toggle in interrupt handler
    GPIO_InitStruct.Pin = GPIO_PIN_6;         // PA6
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Simple output
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM4 - manual toggle in interrupt handler
    GPIO_InitStruct.Pin = GPIO_PIN_9;         // PB9
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Simple output
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    printf("[SYSTEM] GPIO restored: TIM2=AF_OD, TIM1/3/4=OUTPUT_PP\n");
}

void system_init_modes(void)
{
    printf("[SYSTEM] Initializing...\n");

    // ✅ СРАЗУ ПЕРЕХОДИМ В RPM РЕЖИМ вместо BOOT
    g_system_state.current_mode = MODE_RPM_DYNAMIC;  // Было: MODE_BOOT
    g_system_state.channel_mask = 0x0F;  // Все каналы включены по умолчанию
    g_system_state.target_frequency_hz = 0;
    g_system_state.target_frequency_mhz = 0;
    
    // Инициализация значений таймеров
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
    g_system_state.led_last_toggle_time = HAL_GetTick();  // ✅ Инициализируем для RPM режима
    g_system_state.led_state = 0;
    g_system_state.rpm_signal_active = 0;  // ✅ Данных пока нет
    
    // ✅ BOOT LED флаги больше не нужны (или оставь для совместимости)
    g_system_state.led_boot_flashed = 0;
    g_system_state.led_boot_start_time = HAL_GetTick();
    
    // SSR выключен при загрузке
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // PB10 = LOW
    
    // ✅ ТАЙМЕРЫ ЗАПУЩЕНЫ, но выходы будут включены только при RPM > 0x3F
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
    
    // ✅ Включаем выходы (они будут переключаться в прерываниях)
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM4->CCER |= TIM_CCER_CC1E;
    
    printf("[SYSTEM] Started in RPM mode. Waiting for CAN 0x003 data.\n");
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
        restore_gpio_after_external();
    }
    
    g_system_state.rpm_signal_active = 0;
    
    // ============ ПЕРЕКЛЮЧЕНИЕ ============
    
    g_system_state.current_mode = new_mode;
    g_system_state.last_can_command_time = HAL_GetTick();
    
    // ============ ВХОД В НОВЫЙ РЕЖИМ ============
    
    switch(new_mode) {
        
        case MODE_BOOT:
            printf("[SYSTEM] BOOT: LED will flash for 1500ms\n");
            printf("[SYSTEM] NOTE: CAN 0x003 accepted but timers DISABLED\n");
            // В BOOT режиме таймеры ОСТАНОВЛЕНЫ
            // CAN 0x003 будет приниматься, но set_new_speeds() не включит CEN
            break;
        
        case MODE_RPM_DYNAMIC:
            printf("[SYSTEM] RPM MODE: Waiting for CAN 0x003 signals\n");
            printf("[SYSTEM] LED: ON when signal arrives, OFF after 500ms silence\n");
            printf("[SYSTEM] Active channels: 0x%02X\n", g_system_state.channel_mask);
            
            // Включить только те таймеры, которые разрешены маской
            if(g_system_state.channel_mask & 0x01) {
                printf("[SYSTEM] TIM1 (FL) ENABLED\n");
                TIM1->CR1 |= TIM_CR1_CEN;
                TIM1->CCER |= TIM_CCER_CC1E;  // Включить выход
            }
            if(g_system_state.channel_mask & 0x02) {
                printf("[SYSTEM] TIM2 (FR) ENABLED\n");
                // ✅ FIX v4.4: ВОССТАНОВИТЬ PSC=12 для RPM режима
                // FIXED мог поставить PSC=0 для максимальной точности,
                // но calculete_period_only() жёстко предполагает PSC=12
                TIM2->PSC = 12;
                printf("[SYSTEM] TIM2 PSC restored to 12 (RPM requirement)\n");
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM2->CCER |= TIM_CCER_CC1E;  // Включить выход
            }
            if(g_system_state.channel_mask & 0x04) {
                printf("[SYSTEM] TIM3 (RL) ENABLED\n");
                TIM3->CR1 |= TIM_CR1_CEN;
                TIM3->CCER |= TIM_CCER_CC1E;  // Включить выход
            }
            if(g_system_state.channel_mask & 0x08) {
                printf("[SYSTEM] TIM4 (RR) ENABLED\n");
                TIM4->CR1 |= TIM_CR1_CEN;
                TIM4->CCER |= TIM_CCER_CC1E;  // Включить выход
            }
            break;
        
        case MODE_FIXED_FREQUENCY:
            printf("[SYSTEM] FIXED MODE: Frequency = %lu Hz (timer)\n", 
                   g_system_state.target_frequency_hz);
            printf("[SYSTEM] Output frequency = %lu Hz\n",
                   g_system_state.target_frequency_hz / 2);
            printf("[SYSTEM] LED: Constant blink 500ms\n");
            
            // ✅ FIX v4.4: Учёт channel_mask
            if(g_system_state.channel_mask == 0x02) {
                // ---- ONLY_FR: только TIM2 ----
                printf("[SYSTEM] ONLY_FR: TIM2 enabled, TIM1/3/4 stopped\n");
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM2->CCER |= TIM_CCER_CC1E;
                // TIM1/3/4 уже остановлены в секции "ВЫХОД ИЗ СТАРОГО РЕЖИМА"
                
            } else if(g_system_state.channel_mask == 0x0F) {
                // ---- ALL_FOUR: все четыре таймера ----
                printf("[SYSTEM] ALL_FOUR: all timers enabled\n");
                TIM1->CR1 |= TIM_CR1_CEN;
                TIM2->CR1 |= TIM_CR1_CEN;
                TIM3->CR1 |= TIM_CR1_CEN;
                TIM4->CR1 |= TIM_CR1_CEN;
                
                TIM1->CCER |= TIM_CCER_CC1E;
                TIM2->CCER |= TIM_CCER_CC1E;
                TIM3->CCER |= TIM_CCER_CC1E;
                TIM4->CCER |= TIM_CCER_CC1E;
                
            } else {
                printf("[SYSTEM WARNING] Unexpected channel_mask: 0x%02X\n",
                       g_system_state.channel_mask);
            }
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            printf("[SYSTEM] EXTERNAL MODE: GPIO in HIGH-Z, SSR ON for optocoupler\n");
            printf("[SYSTEM] LED: Slow blink 2500ms\n");
            
            // Переводим GPIO в INPUT (HIGH-Z)
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            
            GPIO_InitStruct.Pin = GPIO_PIN_8;   // FL (PA8)
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_15;  // FR (PA15)
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_6;   // RL (PA6)
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            
            GPIO_InitStruct.Pin = GPIO_PIN_9;   // RR (PB9)
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            
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
            // ✅ Одна вспышка 1500ms при загрузке
            if(!g_system_state.led_boot_flashed) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // ON
                g_system_state.led_boot_start_time = current_time;
                g_system_state.led_boot_flashed = 1;
            }
            
            // Выключить через 1500ms
            if(current_time - g_system_state.led_boot_start_time >= 1500) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);    // OFF
            }
            break;
        
        case MODE_RPM_DYNAMIC:
            // ✅ Проверяем таймаут 500ms с момента последних данных
            if(current_time - g_system_state.led_last_toggle_time > 500) {
                // Нет данных >500ms → гасим LED и флаг
                g_system_state.rpm_signal_active = 0;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // OFF
            }
            else if(g_system_state.rpm_signal_active) {
                // ✅ Данные есть → мигаем 100ms
                static uint32_t last_blink = 0;
                if(current_time - last_blink >= 100) {
                    last_blink = current_time;
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
                }
            }
            break;
        
        case MODE_FIXED_FREQUENCY:
            // ✅ Постоянное мигание 500ms
            if(current_time - g_system_state.led_last_toggle_time >= 500) {
                g_system_state.led_last_toggle_time = current_time;
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
            }
            break;
        
        case MODE_EXTERNAL_SIGNAL:
            // ✅ Медленное мигание 2500ms
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

// Новая функция для проверки, должен ли канал работать в текущем режиме
uint8_t is_channel_enabled_in_current_mode(uint8_t channel_num)
{
    if(channel_num >= 4) return 0;
    
    // В BOOT режиме каналы отключены (таймеры остановлены)
    if(g_system_state.current_mode == MODE_BOOT) {
        return 0;
    }
    
    // В RPM режиме проверяем по маске
    if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        return (g_system_state.channel_mask >> channel_num) & 0x01;
    }
    
    // В других режимах (FIXED, EXTERNAL) все каналы работают/не работают по-другому
    return 1;
}
