/**
 * can_commands.c - VERSION 3.2 (DEBUG)
 * 
 * Умный дебаг:
 * - Только критически важные вызовы функций
 * - Ключевые параметры в важных местах
 * - Без засорения вывода
 */

#include "can_commands.h"
#include "system_modes.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include "stm32g4xx_hal.h"

#ifndef APB1_CLK
#define APB1_CLK 150000000
#endif

// Объявление внешних переменных из main.c
extern FDCAN_HandleTypeDef hfdcan1;  // ← ВАЖНО: объявляем hfdcan1

// Локальные прототипы
void send_can_message(uint32_t id, uint8_t* data, uint8_t length);
void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr);
void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr);

// ============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================

/**
 * @brief Обработка выхода из Hi-Z режима для всех команд
 */
static void handle_hi_z_exit_if_needed(void)
{
    // Если активен Hi-Z режим - выходим из него
    if(g_system_state.hi_impedance_active) {
#ifdef DEBUG_CAN_COMMANDS
        printf("[CAN] Exiting Hi-Z mode (was active)\n");
#endif
        exit_hi_impedance_mode();
    }
    
    // Если есть ожидающий Hi-Z вход - отменяем его
    if(g_system_state.pending_hi_z) {
#ifdef DEBUG_CAN_COMMANDS
        printf("[CAN] Cancelling pending Hi-Z\n");
#endif
        g_system_state.pending_hi_z = 0;
    }
}

/**
 * @brief Обновление маски каналов
 */
static void update_channel_mask(uint8_t mask)
{
    /* printf("[CAN] Updating channel mask: 0x%02X -> 0x%02X\n", g_system_state.channel_mask, mask); */
    
    g_system_state.channel_mask = mask;
    
    // Для каналов вне маски гарантированно отключаем
    for(int i = 0; i < 4; i++) {
        if(!(mask & (1 << i))) {
            set_channel_active(i, 0);
        }
    }

    /* printf("[CAN] new channel mask: 0x%02X -> 0x%02X\n", g_system_state.channel_mask, mask); */
    printf("seems like bug was in print!");

}

// ============================================
// ОБРАБОТКА CAN КОМАНД
// ============================================

void process_can_command(uint8_t* data)
{
    if(data == NULL) {
#ifdef DEBUG_CAN_ERROR
        printf("[CAN ERROR] NULL data pointer\n");
#endif
        return;
    }
    
    uint8_t command = data[0];
    
#ifdef DEBUG_CAN_COMMANDS
    printf("[CAN] Processing command: 0x%02X\n", command);
#endif
    
    switch(command) {
        
        // ===== CMD 0x00: DISABLE OUTPUT =====
        case CMD_DISABLE_OUTPUT: {
            uint8_t channel_mask = data[1];
            
#ifdef DEBUG_CAN_COMMANDS
            printf("[CAN] DISABLE_OUTPUT, mask: 0x%02X\n", channel_mask);
#endif
            
            handle_hi_z_exit_if_needed();
            
            // Управление каналами
            if(channel_mask == 0xFF) {
                set_all_channels_active(0);
                update_channel_mask(0x00);
            } else {
                for(int i = 0; i < 4; i++) {
                    if(channel_mask & (1 << i)) {
                        set_channel_active(i, 0);
                    }
                }
                update_channel_mask(0x00);
            }
            
            system_switch_mode(MODE_DISABLED);
            break;
        }
        
        // ===== CMD 0x01: RPM DYNAMIC MODE =====
        case CMD_SET_RPM_MODE: {
            uint8_t channel_mask = data[1];

            printf("[CAN] RPM_MODE, mask: 0x%02X\n", channel_mask);

            handle_hi_z_exit_if_needed();
            
            // Управление каналами
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
                update_channel_mask(0x0F);
            } else {
                for(int i = 0; i < 4; i++) {
                    if(channel_mask & (1 << i)) {
                        set_channel_active(i, 1);
                    }
                }
                update_channel_mask(channel_mask);
            }
            
            system_switch_mode(MODE_RPM_DYNAMIC);
            break;
        }
        
        // ===== CMD 0x02: FIXED FREQUENCY =====
        case CMD_SET_FIXED_FREQ: {
            uint8_t channel_mask = data[1];
            
            // Частота в миллигерцах
            uint32_t freq_mhz = ((uint32_t)data[2]) |
                                ((uint32_t)data[3] << 8) |
                                ((uint32_t)data[4] << 16) |
                                ((uint32_t)data[5] << 24);
            
            float freq_hz = freq_mhz / 1000.0f;
            
            printf("[CAN] FIXED_FREQ: %.3f Hz, mask: 0x%02X\n", freq_hz, channel_mask);
            
            // Проверка диапазона
            if(freq_mhz < 1000 || freq_mhz > 4500000) {
                printf("[CAN ERROR] Frequency out of range: %lu mHz\n", freq_mhz);
                can_tx_error_code = 0x02;
                can_tx_error_pending = 1;
                break;
            }
            
            g_system_state.target_frequency_hz = (uint32_t)freq_hz;
            g_system_state.target_frequency_mhz = freq_mhz;
            
            handle_hi_z_exit_if_needed();
            
            if(channel_mask == 0x02) {
                // ONLY_FR режим
                printf("[CAN] ONLY_FR mode (32-bit TIM2)\n");
                uint16_t psc;
                uint32_t arr_32bit;
                calculate_optimal_psc_arr_32bit(freq_hz, &psc, &arr_32bit);
                
                // Останавливаем другие таймеры
                TIM1->CR1 &= ~TIM_CR1_CEN;
                set_channel_active(0, 0);
                
                TIM3->CR1 &= ~TIM_CR1_CEN;
                set_channel_active(2, 0);
                
                TIM4->CR1 &= ~TIM_CR1_CEN;
                set_channel_active(3, 0);
                
                // Настройка TIM2
                TIM2->CR1 &= ~TIM_CR1_CEN;
                TIM2->PSC = psc;
                TIM2->ARR = arr_32bit;
                TIM2->CNT = 0;
                TIM2->CR1 |= TIM_CR1_CEN;
                
                set_channel_active(1, 1);
                
                // Сохраняем параметры
                g_system_state.psc_values[1] = psc;
                g_system_state.arr_values[1] = (uint16_t)(arr_32bit & 0xFFFF);
                g_system_state.arr_values_32bit[1] = arr_32bit;
                
                update_channel_mask(0x02);
            }
            else if(channel_mask == 0x0F) {
                printf("[CAN] ALL_FOUR mode (16-bit all timers)\n");
                
                uint16_t psc, arr_16bit;
                calculate_optimal_psc_arr_16bit(freq_hz, &psc, &arr_16bit);
                printf("calculated\n");
                
                set_all_channels_active(1);
                printf("all set\n");
                update_channel_mask(0x0F);
                printf("updated\n");
                
                // Настраиваем TIM1, TIM3, TIM4 - 16-битные
                TIM_TypeDef* timers_16bit[3] = {TIM1, TIM3, TIM4};
                
                for(int i = 0; i < 3; i++) {
                    timers_16bit[i]->CR1 &= ~TIM_CR1_CEN;
                    timers_16bit[i]->PSC = psc;
                    timers_16bit[i]->ARR = arr_16bit;
                    timers_16bit[i]->CNT = 0;
                    timers_16bit[i]->CR1 |= TIM_CR1_CEN;
                }

                // TIM2 отдельно - 32-битный (ТОЛЬКО ОДНА НАСТРОЙКА!)
                printf("[FIXED_FREQ] Configuring TIM2 (32-bit)...\n");

                TIM2->DIER &= ~TIM_DIER_UIE;  // Отключить Update Interrupt, иначе виснет

                uint16_t psc_32bit;
                uint32_t arr_32bit;
                calculate_optimal_psc_arr_32bit(freq_hz, &psc_32bit, &arr_32bit);
                printf("[FIXED_FREQ] TIM2 params: PSC=%lu, ARR=%lu\n", (uint32_t)psc_32bit, arr_32bit);

                printf("[TIM2 DIAG] Before config:\n");
                printf("  CR1: 0x%04lX, CEN=%d\n", (uint32_t)TIM2->CR1, (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0);
                printf("  PSC: %lu\n", (uint32_t)TIM2->PSC);
                printf("  ARR: %lu\n", TIM2->ARR);
                printf("  CCR1: %lu\n", TIM2->CCR1);
                printf("  CCER: 0x%04lX\n", (uint32_t)TIM2->CCER);
                printf("  DIER: 0x%04lX\n", (uint32_t)TIM2->DIER);

                printf("[FIXED_FREQ] Stopping TIM2...\n");
                TIM2->CR1 &= ~TIM_CR1_CEN;
                printf("[FIXED_FREQ] TIM2 stopped\n");

                printf("[FIXED_FREQ] Setting TIM2->PSC = %lu\n", (uint32_t)psc_32bit);
                TIM2->PSC = psc_32bit;
                printf("[FIXED_FREQ] TIM2->PSC set\n");

                printf("[FIXED_FREQ] Setting TIM2->ARR = %lu\n", arr_32bit);
                TIM2->ARR = arr_32bit;
                printf("[FIXED_FREQ] TIM2->ARR set\n");

                printf("[FIXED_FREQ] Resetting TIM2->CNT\n");
                TIM2->CNT = 0;

                printf("[FIXED_FREQ] Setting TIM2 PWM duty: CCR1 = %lu\n", arr_32bit / 2);
                TIM2->CCR1 = arr_32bit / 2;

                printf("[FIXED_FREQ] Enabling TIM2 PWM output (CCER)\n");
                TIM2->CCER |= TIM_CCER_CC1E;

                printf("[FIXED_FREQ] Starting TIM2...\n");
                TIM2->CR1 |= TIM_CR1_CEN;
                printf("[FIXED_FREQ] TIM2 started\n");
                
                // Сохраняем параметры
                g_system_state.psc_values[1] = psc_32bit;
                g_system_state.arr_values_32bit[1] = arr_32bit;

                // Диагностика после запуска
                printf("[TIM2 DIAG] After config:\n");
                printf("  CR1: 0x%04lX, CEN=%d\n", (uint32_t)TIM2->CR1, (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0);
                printf("  PSC: %lu\n", (uint32_t)TIM2->PSC);
                printf("  ARR: %lu\n", TIM2->ARR);
                printf("  CCR1: %lu\n", TIM2->CCR1);
                printf("  CCER: 0x%04lX\n", (uint32_t)TIM2->CCER);
                
                printf("done\n");
            }
            else {
#ifdef DEBUG_CAN_ERROR
                printf("[CAN ERROR] Unsupported channel mask: 0x%02X\n", channel_mask);
#endif
                can_tx_error_code = 0x03;
                can_tx_error_pending = 1;
                break;
            }
            
            system_switch_mode(MODE_FIXED_FREQUENCY);
            break;
        }
        
        // ===== CMD 0x06: HI-IMPEDANCE MODE =====
        case CMD_SET_HI_IMPEDANCE: {
            printf("[CAN] HI_IMPEDANCE request\n");
            
            // Проверка состояния
            if(g_system_state.hi_impedance_active) {
                printf("[CAN] Hi-Z already active, ignoring\n");
                can_tx_status_pending = 1;
                break;
            }
            
            if(g_system_state.pending_hi_z) {
                printf("[CAN] Hi-Z already pending, ignoring\n");
                can_tx_status_pending = 1;
                break;
            }
            
            // Устанавливаем флаг
            g_system_state.pending_hi_z = 1;
            can_tx_status_pending = 1;
            
            printf("[CAN] Hi-Z pending flag set\n");
            break;
        }
        
        // ===== CMD 0x07: STATUS REQUEST =====
        case CMD_REQUEST_STATUS: {
            printf("[CAN] STATUS_REQUEST\n");
            
            can_tx_status_pending = 1;
            break;
        }
        
        // ===== НЕИЗВЕСТНАЯ КОМАНДА =====
        default: {
#ifdef DEBUG_CAN_ERROR
            printf("[CAN ERROR] Unknown command: 0x%02X\n", command);
#endif
            can_tx_error_code = 0x01;
            can_tx_error_pending = 1;
            break;
        }
    }
    
    g_system_state.last_can_command_time = HAL_GetTick();
}

// ============================================
// ОТПРАВКА СТАТУСА
// ============================================

void send_system_status(void)
{
#ifdef DEBUG_CAN_TX
    printf("[CAN TX] Sending system status\n");
#endif
    
    uint8_t status_data[8] = {0};
    
    // Byte 0: Текущий режим
    status_data[0] = (uint8_t)g_system_state.current_mode;
    
    // Byte 1: Маска активных каналов
    status_data[1] = g_system_state.channel_mask;
    
    // Bytes 2-3: Частота ×10
    uint16_t freq_x10 = 0;
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        freq_x10 = (uint16_t)(g_system_state.target_frequency_hz * 10);
    }
    status_data[2] = freq_x10 & 0xFF;
    status_data[3] = (freq_x10 >> 8) & 0xFF;
    
    // Byte 4: Флаги состояния
    status_data[4] = 0;
    if(g_system_state.hi_impedance_active) {
        status_data[4] |= 0x02;
    }
    if(g_system_state.pending_hi_z) {
        status_data[4] |= 0x04;
    }
    
    // Bytes 5-6: Uptime
    uint32_t uptime = system_get_uptime_seconds();
    if(uptime > 65535) uptime = 65535;
    status_data[5] = uptime & 0xFF;
    status_data[6] = (uptime >> 8) & 0xFF;
    
    // Byte 7: Reserved
    status_data[7] = 0;
    
    send_can_message(CAN_STATUS_ID, status_data, 8);
}

// ============================================
// ОТПРАВКА CAN СООБЩЕНИЯ
// ============================================

void send_can_message(uint32_t id, uint8_t* data, uint8_t length)
{
#ifdef DEBUG_CAN_TX
    printf("[CAN TX] ID: 0x%03lX, Len: %d\n", id, length);
    // или: printf("[CAN TX] ID: 0x%03X, Len: %d\n", (unsigned int)id, length);
#endif
    
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
#ifdef DEBUG_CALC
    printf("[CALC_32BIT] Start: %.3f Hz\n", freq_hz);
#endif
    
    if(freq_hz <= 0 || freq_hz > APB1_CLK) {
        *psc = 0;
        *arr = 1;
#ifdef DEBUG_CALC
        printf("[CALC_32BIT] ERROR: Invalid frequency\n");
#endif
        return;
    }
    
    uint64_t target_ticks = (uint64_t)(APB1_CLK / freq_hz);
    
    uint64_t best_error = 0xFFFFFFFFFFFFFFFFULL;
    uint16_t best_psc = 0;
    uint32_t best_arr = 1;
    
    for(uint32_t p = 0; p <= 65535; p++) {
        uint64_t arr_calc = (target_ticks / (p + 1)) - 1;
        
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
    }
    
    *psc = best_psc;
    *arr = best_arr;
    
#ifdef DEBUG_CALC
    float actual_freq = (float)APB1_CLK / ((best_psc + 1) * (best_arr + 1));
    float error_pct = fabsf(actual_freq - freq_hz) / freq_hz * 100.0f;
    printf("[CALC_32BIT] Result: PSC=%u, ARR=%lu (%.3f Hz, error: %.4f%%)\n",
           best_psc, best_arr, actual_freq, error_pct);
#endif
}

// ============================================
// РАСЧЁТ PSC/ARR ДЛЯ 16-БИТНЫХ ТАЙМЕРОВ
// ============================================

void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr)
{
#ifdef DEBUG_CALC
    printf("[CALC_16BIT] Start: %.3f Hz\n", freq_hz);
#endif
    
    if(freq_hz <= 0 || freq_hz > APB1_CLK) {
        *psc = 0;
        *arr = 1;
#ifdef DEBUG_CALC
        printf("[CALC_16BIT] ERROR: Invalid frequency\n");
#endif
        return;
    }
    
    uint32_t target_ticks = (uint32_t)(APB1_CLK / freq_hz);
    
    uint32_t best_error = 0xFFFFFFFF;
    uint16_t best_psc = 0;
    uint16_t best_arr = 1;
    
    for(uint32_t p = 0; p <= 65535; p++) {
        uint32_t arr_calc = (target_ticks / (p + 1)) - 1;
        
        if(arr_calc > 65535) continue;
        if(arr_calc < 1) continue;
        
        uint32_t actual_ticks = (p + 1) * (arr_calc + 1);
        uint32_t error = (actual_ticks > target_ticks) ?
                        (actual_ticks - target_ticks) :
                        (target_ticks - actual_ticks);
        
        if(error < best_error) {
            best_error = error;
            best_psc = (uint16_t)p;
            best_arr = (uint16_t)arr_calc;
        }
    }
    
    *psc = best_psc;
    *arr = best_arr;
    
#ifdef DEBUG_CALC
    float actual_freq = (float)APB1_CLK / ((best_psc + 1) * (best_arr + 1));
    float error_pct = fabsf(actual_freq - freq_hz) / freq_hz * 100.0f;
    printf("[CALC_16BIT] Result: PSC=%u, ARR=%u (%.3f Hz, error: %.4f%%)\n",
           best_psc, best_arr, actual_freq, error_pct);
#endif
}

// ============================================
// ОТПРАВКА ОШИБКИ
// ============================================

void send_error_response(uint8_t error_code)
{
#ifdef DEBUG_CAN_TX
    printf("[CAN TX ERROR] Sending error code: 0x%02X\n", error_code);
#endif
    
    uint8_t error_data[8] = {0};
    error_data[0] = 0xFF;
    error_data[1] = error_code;
    
    send_can_message(CAN_STATUS_ID, error_data, 8);
}
