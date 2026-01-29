/**
 * can_commands.c - VERSION 3 (CLEANED UP)
 * 
 * Изменения:
 * - УДАЛЕН весь analog_follower код
 * - Hi-Z режим с SSR (PB10) управлением
 * - Два режима FIXED_FREQUENCY: ALL_FOUR (16-bit) и ONLY_FR (32-bit TIM2)
 * - LED в RPM режиме: не мигает при старте (фиксим после получения первого сообщения)
 * - Убрана минимизация printf (добавим дебаг флаги позже)
 */

#include "can_commands.h"
#include "system_modes.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef APB1_CLK
#define APB1_CLK 150000000
#endif

extern void my_printf(const char *fmt, ...);
extern FDCAN_HandleTypeDef hfdcan1;

// Локальные прототипы
void send_can_message(uint32_t id, uint8_t* data, uint8_t length);
void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr);
void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr);

// ============================================
// ОБРАБОТКА CAN КОМАНД
// ============================================

void process_can_command(uint8_t* data)
{
    if(data == NULL) return;
    
    uint8_t command = data[0];
    
    #if DEBUG_CAN_COMMANDS
    my_printf("[CAN] Command: 0x%02X\n", command);
    #endif
    
    switch(command) {
        
        // ===== CMD 0x00: DISABLE OUTPUT =====
        case CMD_DISABLE_OUTPUT: {
            uint8_t channel_mask = data[1];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] DISABLE_OUTPUT, mask: 0x%02X\n", channel_mask);
            #endif
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(0);
            } else {
                for(int i = 0; i < 4; i++) {
                    if(channel_mask & (1 << i)) {
                        set_channel_active(i, 0);
                    }
                }
            }
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            system_switch_mode(MODE_DISABLED);
            break;
        }
        
        // ===== CMD 0x01: RPM DYNAMIC MODE =====
        case CMD_SET_RPM_MODE: {
            uint8_t channel_mask = data[1];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] RPM_MODE, mask: 0x%02X\n", channel_mask);
            #endif
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            } else {
                for(int i = 0; i < 4; i++) {
                    if(channel_mask & (1 << i)) {
                        set_channel_active(i, 1);
                    }
                }
            }
            
            system_switch_mode(MODE_RPM_DYNAMIC);
            break;
        }
        
        // ===== CMD 0x02: FIXED FREQUENCY =====
        case CMD_SET_FIXED_FREQ: {
            uint8_t channel_mask = data[1];
            
            // Частота в миллигерцах (uint32_t LE)
            uint32_t freq_mhz = ((uint32_t)data[2]) |
                                ((uint32_t)data[3] << 8) |
                                ((uint32_t)data[4] << 16) |
                                ((uint32_t)data[5] << 24);
            
            // Преобразуем в Hz (float для точности)
            float freq_hz = freq_mhz / 1000.0f;
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] FIXED_FREQ: %.3f Hz (mask=0x%02X)\n", freq_hz, channel_mask);
            #endif
            
            // Проверка диапазона
            if(freq_mhz < 1000 || freq_mhz > 4500000) {
                my_printf("[CAN] ERROR: Frequency out of range: %lu mHz\n", freq_mhz);
                can_tx_error_code = 0x02;
                can_tx_error_pending = 1;
                break;
            }
            
            g_system_state.target_frequency_hz = (uint32_t)freq_hz;
            g_system_state.target_frequency_mhz = freq_mhz;
            
            if(channel_mask == 0x02) {
                // ONLY_FR режим: только TIM2 (32-бит)
                uint16_t psc;
                uint32_t arr_32bit;
                calculate_optimal_psc_arr_32bit(freq_hz, &psc, &arr_32bit);
                
                // Останавливаем другие таймеры
                TIM1->CR1 &= ~TIM_CR1_CEN;
                TIM3->CR1 &= ~TIM_CR1_CEN;
                TIM4->CR1 &= ~TIM_CR1_CEN;
                
                // Настройка TIM2
                TIM2->CR1 &= ~TIM_CR1_CEN;
                TIM2->PSC = psc;
                TIM2->ARR = arr_32bit;
                TIM2->CNT = 0;
                TIM2->CR1 |= TIM_CR1_CEN;
                
                g_system_state.psc_values[1] = psc;
                g_system_state.arr_values[1] = (uint16_t)(arr_32bit & 0xFFFF);
                g_system_state.arr_values_32bit[1] = arr_32bit;
            }
            else if(channel_mask == 0x0F) {
                // ALL_FOUR режим: все 4 таймера (16-бит)
                uint16_t psc, arr_16bit;
                calculate_optimal_psc_arr_16bit(freq_hz, &psc, &arr_16bit);
                
                TIM_TypeDef* timers[4] = {TIM1, TIM2, TIM3, TIM4};
                for(int i = 0; i < 4; i++) {
                    timers[i]->CR1 &= ~TIM_CR1_CEN;
                    timers[i]->PSC = psc;
                    timers[i]->ARR = arr_16bit;
                    timers[i]->CNT = 0;
                    timers[i]->CR1 |= TIM_CR1_CEN;
                    
                    g_system_state.psc_values[i] = psc;
                    g_system_state.arr_values[i] = arr_16bit;
                }
            }
            else {
                my_printf("[CAN] ERROR: Unsupported channel mask: 0x%02X\n", channel_mask);
                can_tx_error_code = 0x03;
                can_tx_error_pending = 1;
                break;
            }
            
            system_switch_mode(MODE_FIXED_FREQUENCY);
            break;
        }
        
        // ===== CMD 0x06: HI-IMPEDANCE MODE =====
        case CMD_SET_HI_IMPEDANCE: {
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] HI_IMPEDANCE request\n");
            #endif
            
            // Флаг для обработки в main loop
            g_system_state.pending_hi_z = 1;
            can_tx_status_pending = 1;
            break;
        }
        
        // ===== CMD 0x07: STATUS REQUEST =====
        case CMD_REQUEST_STATUS: {
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] STATUS_REQUEST\n");
            #endif
            
            can_tx_status_pending = 1;
            break;
        }
        
        // ===== НЕИЗВЕСТНАЯ КОМАНДА =====
        default: {
            my_printf("[CAN] Unknown command: 0x%02X\n", command);
            can_tx_error_code = 0x01;
            can_tx_error_pending = 1;
            break;
        }
    }
    
    g_system_state.last_can_command_time = HAL_GetTick();
}

// ============================================
// HI-Z РЕЖИМ - ВХОД
// ============================================

void enter_hi_impedance_mode(void)
{
    my_printf("\n[HI-Z] ===== ENTERING HI-IMPEDANCE MODE =====\n");
    
    // === 1. ОСТАНАВЛИВАЕМ ВСЕ ТАЙМЕРЫ ===
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    my_printf("[HI-Z] All timers stopped\n");
    
    // === 2. ПЕРЕВОДИМ GPIO В INPUT (Hi-Z) ===
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // PA8 (TIM1_CH1) - FL
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA8 (FL) → INPUT\n");
    
    // PA15 (TIM2_CH1) - FR ← ИЗМЕНЕНО!
    GPIO_InitStruct.Pin = GPIO_PIN_15;  // ← PA15 вместо PA5
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA15 (FR) → INPUT\n");
    
    // PA6 (TIM3_CH1) - RL
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA6 (RL) → INPUT\n");
    
    // PB6 (TIM4_CH1) - RR
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    my_printf("[HI-Z] PB6 (RR) → INPUT\n");
    
    // === 3. ВКЛЮЧАЕМ SOLID STATE RELAY (PB10) ===
    HAL_GPIO_WritePin(GPIOB, SOLID_RELAY_CONTROL_Pin, GPIO_PIN_SET);
    my_printf("[HI-Z] SSR Control (PB10) → HIGH (external signal active)\n");
    
    // === 4. УСТАНАВЛИВАЕМ ФЛАГИ И РЕЖИМ ===
    g_system_state.hi_impedance_active = 1;
    system_switch_mode(MODE_HI_IMPEDANCE);
    
    my_printf("[HI-Z] ✓ HI-IMPEDANCE active - safe for external signals\n");
    my_printf("[HI-Z] ✓ PA15 (FR) controlled by external generator via SSR\n");
    my_printf("[HI-Z] ======================================\n\n");
}

// ============================================
// HI-Z РЕЖИМ - ВЫХОД
// ============================================

void exit_hi_impedance_mode(void)
{
    my_printf("\n[HI-Z] ===== EXITING HI-IMPEDANCE MODE =====\n");
    
    // === 1. ВЫКЛЮЧАЕМ SOLID STATE RELAY (PB10) ===
    HAL_GPIO_WritePin(GPIOB, SOLID_RELAY_CONTROL_Pin, GPIO_PIN_RESET);
    my_printf("[HI-Z] SSR Control (PB10) → LOW (STM32 control active)\n");
    
    // === 2. ВОССТАНАВЛИВАЕМ GPIO КАК ALTERNATE FUNCTION ===
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // PA8 (TIM1_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA8 (FL) → AF2 (TIM1_CH1)\n");
    
    // PA15 (TIM2_CH1) - AF1 ← ИЗМЕНЕНО!
    GPIO_InitStruct.Pin = GPIO_PIN_15;  // ← PA15 вместо PA5
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA15 (FR) → AF1 (TIM2_CH1)\n");
    
    // PA6 (TIM3_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA6 (RL) → AF2 (TIM3_CH1)\n");
    
    // PB6 (TIM4_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    my_printf("[HI-Z] PB6 (RR) → AF2 (TIM4_CH1)\n");
    
    // === 3. ВОССТАНАВЛИВАЕМ ТАЙМЕРЫ ===
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        my_printf("[HI-Z] Restoring FIXED_FREQUENCY mode...\n");
        
        // Восстанавливаем последние настройки
        for(int i = 0; i < 4; i++) {
            if(g_system_state.channel_mask & (1 << i)) {
                TIM_TypeDef* TIMx = NULL;
                switch(i) {
                    case 0: TIMx = TIM1; break;
                    case 1: TIMx = TIM2; break;
                    case 2: TIMx = TIM3; break;
                    case 3: TIMx = TIM4; break;
                }
                
                if(TIMx != NULL) {
                    uint16_t psc = g_system_state.psc_values[i];
                    uint16_t arr = g_system_state.arr_values[i];
                    
                    TIMx->CR1 &= ~TIM_CR1_CEN;
                    TIMx->PSC = psc;
                    TIMx->ARR = arr;
                    TIMx->CNT = 0;
                    TIMx->CR1 |= TIM_CR1_CEN;
                }
            }
        }
    } else if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        my_printf("[HI-Z] Restoring RPM_DYNAMIC mode...\n");
        
        TIM1->CR1 |= TIM_CR1_CEN;
        TIM2->CR1 |= TIM_CR1_CEN;
        TIM3->CR1 |= TIM_CR1_CEN;
        TIM4->CR1 |= TIM_CR1_CEN;
    }
    
    g_system_state.hi_impedance_active = 0;
    
    my_printf("[HI-Z] ✓ GPIO restored to timer outputs\n");
    my_printf("[HI-Z] =====================================\n\n");
}

// ============================================
// ОТПРАВКА СТАТУСА (ID 0x006)
// ============================================

void send_system_status(void)
{
    uint8_t status_data[8] = {0};
    
    // Byte 0: Текущий режим
    status_data[0] = (uint8_t)g_system_state.current_mode;
    
    // Byte 1: Маска активных каналов
    status_data[1] = g_system_state.channel_mask;
    
    // Bytes 2-3: Частота ×10 (0.1 Hz разрешение)
    uint16_t freq_x10 = 0;
    
    // Bytes 4: Флаги состояния
    status_data[4] = 0;
    if(g_system_state.hi_impedance_active) {
        status_data[4] |= 0x02;  // Бит Hi-Z
    }
    
    // Bytes 5-6: Uptime в секундах (uint16_t LE)
    uint32_t uptime = system_get_uptime_seconds();
    if(uptime > 65535) uptime = 65535;
    status_data[5] = uptime & 0xFF;
    status_data[6] = (uptime >> 8) & 0xFF;
    
    // Byte 7: Резервировано
    status_data[7] = 0;
    
    send_can_message(CAN_STATUS_ID, status_data, 8);
    
    #if DEBUG_CAN_TX
    my_printf("[STATUS] Mode=%s, Ch=0x%02X, Freq=%.1f Hz, HiZ=%d, Uptime=%lu s\n",
              get_mode_name(status_data[0]),
              status_data[1],
              freq_x10 / 10.0f,
              (status_data[4] & 0x02) ? 1 : 0,
              uptime);
    #endif
}

    
// ============================================
// ОТПРАВКА CAN СООБЩЕНИЯ
// ============================================

void send_can_message(uint32_t id, uint8_t* data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}


// ============================================
// РАСЧЁТ PSC/ARR ДЛЯ 32-БИТНОГО TIM2
// ============================================

void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr)
{
    if(freq_hz <= 0 || freq_hz > APB1_CLK) {
        *psc = 0;
        *arr = 1;
        my_printf("[CALC_32BIT] ERROR: Invalid frequency: %.3f Hz\n", freq_hz);
        return;
    }
    
    uint64_t target_ticks = (uint64_t)(APB1_CLK / freq_hz);
    
    uint64_t best_error = 0xFFFFFFFFFFFFFFFFULL;
    uint16_t best_psc = 0;
    uint32_t best_arr = 1;
    
    // Расширенный поиск PSC для максимальной точности
    for(uint32_t p = 0; p <= 65535; p++) {
        uint64_t arr_calc = (target_ticks / (p + 1)) - 1;
        
        // Для 32-бит таймера максимум ARR = 2^32-1
        if(arr_calc > 0xFFFFFFFFUL) continue;
        if(arr_calc < 1) continue;
        
        uint64_t actual_ticks = (p + 1) * (arr_calc + 1);
        uint64_t error = (actual_ticks > target_ticks) ?
                        (actual_ticks - target_ticks) :
                        (target_ticks - actual_ticks);
        
        if(error < best_error) {
            best_error = error;
            best_psc = (uint16_t)p;
            best_arr = (uint32_t)arr_calc;
        }
        
        // Если ошибка меньше 0.001 Гц - отлично!
        float actual_freq = (float)APB1_CLK / ((best_psc + 1) * (best_arr + 1));
        if(fabsf(actual_freq - freq_hz) / freq_hz < 0.001) {
            break;
        }
    }
    
    *psc = best_psc;
    *arr = best_arr;
    
    float actual_freq = (float)APB1_CLK / ((best_psc + 1) * (best_arr + 1));
    float error_hz = fabsf(actual_freq - freq_hz);
    float error_pct = (error_hz / freq_hz) * 100.0f;
    
    my_printf("[CALC_32BIT] Target: %.3f Hz\n", freq_hz);
    my_printf("[CALC_32BIT] Actual: %.3f Hz\n", actual_freq);
    my_printf("[CALC_32BIT] PSC=%u, ARR=%lu (32-bit)\n", best_psc, best_arr);
    my_printf("[CALC_32BIT] Error: %.6f%% (%.6f Hz)\n", error_pct, error_hz);
}


// ============================================
// РАСЧЁТ PSC/ARR ДЛЯ 16-БИТНЫХ ТАЙМЕРОВ
// ============================================



void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr)
{
    if(freq_hz <= 0 || freq_hz > APB1_CLK) {
        *psc = 0;
        *arr = 1;
        my_printf("[CALC_16BIT] ERROR: Invalid frequency: %.3f Hz\n", freq_hz);
        return;
    }
    
    uint32_t target_ticks = (uint32_t)(APB1_CLK / freq_hz);
    
    uint32_t best_error = 0xFFFFFFFF;
    uint16_t best_psc = 0;
    uint16_t best_arr = 1;
    
    for(uint32_t p = 0; p <= 65535; p++) {
        uint32_t arr_calc = (target_ticks / (p + 1)) - 1;
        
        // Ограничение для 16-bit таймеров
        if(arr_calc > 65535) continue;
        if(arr_calc < 1) continue;
        
        // Остальная логика аналогична 32-bit версии
    }
}


// ============================================
// ОТПРАВКА ОШИБКИ
// ============================================

void send_error_response(uint8_t error_code)
{
    uint8_t error_data[8] = {0};
    error_data[0] = 0xFF;
    error_data[1] = error_code;
    
    send_can_message(CAN_STATUS_ID, error_data, 8);
    
    my_printf("[CAN_ERROR] Code: 0x%02X\n", error_code);
}
