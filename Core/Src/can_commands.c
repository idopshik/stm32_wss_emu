
/**
 * can_commands.c - VERSION 4.3 (FIXED STATUS FREQUENCY)
 * 
 * Команды:
 * 0x01 = RPM MODE (динамический)
 * 0x02 = FIXED MODE (статичный)
 * 0x03 = EXTERNAL MODE (GPIO HIGH-Z)
 * 
 * ИСПРАВЛЕНИЕ (v4.3):
 *   send_system_status() возвращал частоту ТАЙМЕРА вместо выходной частоты GPIO.
 *   target_frequency_mhz хранит частоту таймера в миллигерцах
 *   (= output_Hz × 2 × 1000).
 *   Статус байты 2-3 должны содержать output_Hz × 10:
 *     freq_x10 = target_frequency_mhz / 200
 *              = (output_Hz × 2 × 1000) / 200 = output_Hz × 10  ✓
 *
 *   Старый (неверный) код:
 *     freq_x10 = target_frequency_hz * 10   (= timer_Hz × 10, в 2 раза больше)
 */

#include "can_commands.h"
#include "system_modes.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stm32g4xx_hal.h"

#define APB1_CLK 150000000

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void send_can_message(uint32_t id, uint8_t* data, uint8_t length);
void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr);
void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr);

// ============================================
// ОБРАБОТКА CAN КОМАНД
// ============================================

void process_can_command(uint8_t* data)
{
    if(data == NULL) {
        my_printf("[CAN ERROR] NULL data\n");
        return;
    }
    
    uint8_t command = data[0];
    my_printf("[CAN] Command: 0x%02X\n", command);
    
    switch(command) {
        
        // ===== 0x01: RPM MODE =====
        case 0x01: {
            uint8_t channel_mask = data[1];
            
            my_printf("[CAN] RPM MODE, channels: 0x%02X\n", channel_mask);
            
            // Обновляем маску
            g_system_state.channel_mask = (channel_mask == 0xFF) ? 0x0F : channel_mask;
            
            // Переходим в режим
            system_switch_mode(MODE_RPM_DYNAMIC);
            
            can_tx_status_pending = 1;
            break;
        }
        
        // ===== 0x02: FIXED MODE =====
        case 0x02: {
            uint8_t channel_mask = data[1];
            
            // Freq_Timer: частота ТАЙМЕРА в миллигерцах (uint32_t LE)
            //   = Желаемая_выходная_частота_Гц × 2 × 1000
            uint32_t freq_mhz = ((uint32_t)data[2]) |
                                ((uint32_t)data[3] << 8) |
                                ((uint32_t)data[4] << 16) |
                                ((uint32_t)data[5] << 24);
            
            // freq_hz здесь — это частота ТАЙМЕРА (не выходная)
            float freq_hz = freq_mhz / 1000.0f;
            
            my_printf("[CAN] FIXED MODE: timer=%.3f Hz (out=%.3f Hz), channels: 0x%02X\n",
                      freq_hz, freq_hz / 2.0f, channel_mask);
            
            if(freq_mhz < 1000 || freq_mhz > 4500000) {
                my_printf("[CAN ERROR] Frequency out of range: %lu mHz\n", freq_mhz);
                break;
            }
            
            // Сохраняем частоту таймера — send_system_status разделит на 2 при отправке
            g_system_state.target_frequency_hz = (uint32_t)freq_hz;   // timer Hz
            g_system_state.target_frequency_mhz = freq_mhz;           // timer mHz
            g_system_state.channel_mask = (channel_mask == 0xFF) ? 0x0F : channel_mask;
            
            // Настроить таймеры
            if(channel_mask == 0x02) {
                // ONLY_FR: 32-бит TIM2
                uint16_t psc;
                uint32_t arr_32bit;
                calculate_optimal_psc_arr_32bit(freq_hz, &psc, &arr_32bit);
                
                TIM2->PSC = psc;
                TIM2->ARR = arr_32bit;
                TIM2->CNT = 0;
            }
            else if(channel_mask == 0x0F) {
                // ALL_FOUR: все 16-бит таймеры
                uint16_t psc, arr_16bit;
                calculate_optimal_psc_arr_16bit(freq_hz, &psc, &arr_16bit);
                
                TIM1->PSC = psc;
                TIM1->ARR = arr_16bit;
                TIM1->CNT = 0;
                
                TIM3->PSC = psc;
                TIM3->ARR = arr_16bit;
                TIM3->CNT = 0;
                
                TIM4->PSC = psc;
                TIM4->ARR = arr_16bit;
                TIM4->CNT = 0;
                
                // TIM2 отдельно
                uint16_t psc_32;
                uint32_t arr_32;
                calculate_optimal_psc_arr_32bit(freq_hz, &psc_32, &arr_32);
                
                TIM2->PSC = psc_32;
                TIM2->ARR = arr_32;
                TIM2->CNT = 0;
            }
            
            // Переходим в режим
            system_switch_mode(MODE_FIXED_FREQUENCY);
            
            can_tx_status_pending = 1;
            break;
        }
        
        // ===== 0x03: EXTERNAL MODE =====
        case 0x03: {
            my_printf("[CAN] EXTERNAL MODE\n");
            
            // Переходим в режим (GPIO будут переведены в HIGH-Z)
            system_switch_mode(MODE_EXTERNAL_SIGNAL);
             
            can_tx_status_pending = 1;
            break;
        }

        case 0x07: {
            my_printf("[CAN] STATUS REQUEST\n");
            can_tx_status_pending = 1;
            break;
        }
        
        default: {
            my_printf("[CAN ERROR] Unknown command: 0x%02X\n", command);
            can_tx_error_pending = 1;
            can_tx_error_code = 0x01;    // код ошибки "неизвестная команда"
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
    HAL_Delay(25);  // 25ms turnaround time
    my_printf("[CAN TX] Status\n");
    
    uint8_t status_data[8] = {0};
    
    status_data[0] = (uint8_t)g_system_state.current_mode;
    status_data[1] = g_system_state.channel_mask;
    
    // Байты 2-3: Выходная частота × 10 (только для FIXED)
    //
    // target_frequency_mhz = timer_Hz × 1000 = output_Hz × 2 × 1000
    //
    // FIX: делим на 200, чтобы получить output_Hz × 10:
    //   target_frequency_mhz / 200 = (output_Hz × 2000) / 200 = output_Hz × 10  ✓
    //
    // Старый (неверный) код использовал target_frequency_hz * 10:
    //   target_frequency_hz = timer_Hz = output_Hz × 2
    //   => output_Hz × 2 × 10 = output_Hz × 20  ← в 2 раза больше нужного
    //
    // Пример: output = 1000 Hz, freq_mhz = 2 000 000
    //   ИСПРАВЛЕННЫЙ:  2 000 000 / 200 = 10 000 → 1000.0 Hz  ✓
    //   СТАРЫЙ:        2000 * 10        = 20 000 → 2000.0 Hz  ✗
    uint16_t freq_x10 = 0;
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        freq_x10 = (uint16_t)(g_system_state.target_frequency_mhz / 200);
    }
    status_data[2] = freq_x10 & 0xFF;
    status_data[3] = (freq_x10 >> 8) & 0xFF;
    
    // Байт 4: Флаги (бит 1 = Hi-Z active)
    status_data[4] = 0;
    
    // Байт 5: Reserved
    status_data[5] = 0;
    
    // Байты 6-7: Uptime seconds (uint16_t LE)
    uint32_t uptime = system_get_uptime_seconds();
    if(uptime > 65535) uptime = 65535;
    status_data[6] = uptime & 0xFF;
    status_data[7] = (uptime >> 8) & 0xFF;
    
    send_can_message(0x006, status_data, 8);
}

// ============================================
// ОТПРАВКА CAN
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
// РАСЧЁТ PSC/ARR
// ============================================

void calculate_optimal_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr)
{
    if(freq_hz <= 0) {
        *psc = 0;
        *arr = 1;
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
}

void calculate_optimal_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr)
{
    if(freq_hz <= 0) {
        *psc = 0;
        *arr = 1;
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
}

// ============================================
// ОТПРАВКА ОШИБКИ
// ============================================

void send_error_response(uint8_t error_code)
{
    my_printf("[CAN TX ERROR] Sending error code: 0x%02X\n", error_code);
    
    uint8_t error_data[8] = {0};
    error_data[0] = 0xFF;
    error_data[1] = error_code;
    
    send_can_message(CAN_STATUS_ID, error_data, 8);
}
