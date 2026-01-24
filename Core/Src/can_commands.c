
#include "can_commands.h"
#include "system_modes.h"
#include <string.h>

// Используем APB1_CLK из main.c
#ifndef APB1_CLK
#define APB1_CLK 150000000  // 150 МГц по умолчанию

#endif


extern void my_printf(const char *fmt, ...);

extern FDCAN_HandleTypeDef hfdcan1;  // Из main.c

// Прототипы локальных функций
void set_fixed_frequency_to_timers(uint32_t freq_hz, uint16_t psc, uint16_t arr, uint8_t channel_mask);
void send_can_message(uint32_t id, uint8_t* data, uint8_t length);

extern FDCAN_HandleTypeDef hfdcan1;  // Из main.c

// ============================================
// ОБРАБОТКА КОМАНД
// ============================================

void process_can_command(uint8_t* data)
{
    if(data == NULL) return;
    
    uint8_t command = data[0];
    uint8_t channel_mask = data[1];
    
    my_printf("[CAN] Processing command: 0x%02X, channels: 0x%02X\n", 
              command, channel_mask);
    
    switch(command) {
        case CMD_DISABLE_OUTPUT: {
            my_printf("[CAN] Command: DISABLE OUTPUT\n");
            if(channel_mask == 0xFF) {
                set_all_channels_active(0);
            } else {
                for(int i = 0; i < 4; i++) {
                    if(channel_mask & (1 << i)) {
                        set_channel_active(i, 0);
                    }
                }
            }
            system_switch_mode(MODE_DISABLED);
            break;
        }
        
        case CMD_SET_RPM_MODE: {
            my_printf("[CAN] Command: SET RPM MODE\n");
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            system_switch_mode(MODE_RPM_DYNAMIC);
            break;
        }
        
        case CMD_SET_FIXED_FREQ: {
            // Байты 2-5: частота в Гц (uint32_t, little-endian)
            uint32_t freq_hz = ((uint32_t)data[5] << 24) |
                               ((uint32_t)data[4] << 16) |
                               ((uint32_t)data[3] << 8) |
                               data[2];
            
            // Байты 6-7: PSC значение (если 0xFFFF - авто)
            uint16_t psc_value = ((uint16_t)data[7] << 8) | data[6];
            uint16_t arr_value;
            
            my_printf("[CAN] Command: SET FIXED FREQ = %lu Hz, PSC = %u\n", 
                      freq_hz, psc_value);
            
            g_system_state.target_frequency_hz = freq_hz;
            
            // Если PSC = 0xFFFF, используем авто-подбор
            if(psc_value == 0xFFFF) {
                calculate_optimal_psc_arr(freq_hz, &psc_value, &arr_value);
            } else {
                // Рассчитываем ARR для заданного PSC
                arr_value = (APB1_CLK / (freq_hz * (psc_value + 1))) - 1;
                if(arr_value > 65535) arr_value = 65535;
                if(arr_value < 1) arr_value = 1;
            }
            
            // Устанавливаем фиксированную частоту
            set_fixed_frequency_to_timers(freq_hz, psc_value, arr_value, data[1]);
            
            // Сохраняем в глобальную структуру
            for(int i = 0; i < 4; i++) {
                if(data[1] & (1 << i)) {
                    g_system_state.psc_values[i] = psc_value;
                    g_system_state.arr_values[i] = arr_value;
                }
            }
            
            // Переключаем режим
            system_switch_mode(MODE_FIXED_FREQUENCY);
            
            my_printf("[CAN] Fixed frequency mode activated\n");
            break;
        }
        
    case CMD_SET_PWM_MODE: {
        // Байты 2-5: частота в Гц
        uint32_t freq_hz = ((uint32_t)data[5] << 24) |
                           ((uint32_t)data[4] << 16) |
                           ((uint32_t)data[3] << 8) |
                           data[2];
        
        // Байты 6-7: скважность 0-100% (uint16_t)
        uint16_t duty = ((uint16_t)data[7] << 8) | data[6];
        
        my_printf("[CAN] Command: SET PWM = %lu Hz, Duty = %u%%\n", 
                  freq_hz, duty);
        
        if(duty > 100) duty = 100;
        
        g_system_state.target_frequency_hz = freq_hz;
        g_system_state.pwm_duty_percent = (uint8_t)duty;
        
        // Устанавливаем на таймеры (пока только на TIM2, который у нас в режиме OC)
        // Для других таймеров нужно настраивать ШИМ
        if(freq_hz > 0) {
            // Рассчитываем PSC и ARR
            uint32_t psc = 1499;  // Для 100 Гц
            uint32_t arr = 99;    // Для 100 Гц
            
            if(freq_hz == 1000) {
                psc = 1499;
                arr = 99;
            }
            // Можно добавить расчет для других частот
            
            // Устанавливаем на TIM2 (он у нас в режиме Output Compare)
            TIM2->CR1 &= ~TIM_CR1_CEN;
            TIM2->PSC = psc;
            TIM2->ARR = arr;
            TIM2->CCR1 = (arr * duty) / 100;  // Устанавливаем скважность
            TIM2->CNT = 0;
            TIM2->CR1 |= TIM_CR1_CEN;
            
            my_printf("[CAN] PWM set on TIM2: PSC=%lu, ARR=%lu, CCR1=%lu\n", 
                      psc, arr, TIM2->CCR1);
        }
        
        if(channel_mask == 0xFF) {
            set_all_channels_active(1);
        }
        
        system_switch_mode(MODE_PWM);
        break;
    }
        
        case CMD_SET_ANALOG_FOLLOW: {
            my_printf("[CAN] Command: SET ANALOG FOLLOW MODE\n");
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            system_switch_mode(MODE_ANALOG_FOLLOW);
            break;
        }
        
        case CMD_REQUEST_STATUS: {
            my_printf("[CAN] Command: REQUEST STATUS\n");
            send_system_status();
            break;
        }
        
        default: {
            my_printf("[CAN] ERROR: Unknown command: 0x%02X\n", command);
            send_error_response(0x01); // Неизвестная команда
            break;
        }
    }
    
    // Обновляем время последней команды
    g_system_state.last_can_command_time = HAL_GetTick();
}

// ============================================
// ОТПРАВКА СТАТУСА СИСТЕМЫ (ID 0x006)
// ============================================

// В can_commands.c - функция send_system_status
void send_system_status(void)
{
    uint8_t status_data[8] = {0};
    
    // Байт 0: Текущий режим
    status_data[0] = (uint8_t)g_system_state.current_mode;
    
    // Байт 1: Маска активных каналов
    status_data[1] = g_system_state.channel_mask;
    
    // Байты 2-3: Частота / 10 Гц (если установлена)
    if(g_system_state.target_frequency_hz > 0) {
        uint16_t freq_x10 = (uint16_t)(g_system_state.target_frequency_hz / 10);
        status_data[2] = freq_x10 & 0xFF;
        status_data[3] = (freq_x10 >> 8) & 0xFF;
    }
    
    // Байты 4-5: Скважность ШИМ (если режим PWM)
    if(g_system_state.current_mode == MODE_PWM) {
        status_data[4] = g_system_state.pwm_duty_percent;
    }
    
    // Байты 6-7: Наличие аналогового сигнала
    status_data[6] = g_system_state.analog_signal_present;
    
    // Отправляем CAN сообщение
    send_can_message(CAN_STATUS_ID, status_data, 8);
    
    my_printf("[CAN] Status sent: mode=%u, channels=0x%02X\n",
              status_data[0], status_data[1]);
}

// ============================================
// ОТПРАВКА ОШИБКИ
// ============================================

void send_error_response(uint8_t error_code)
{
    uint8_t error_data[8] = {0};
    
    error_data[0] = 0xFF;  // Флаг ошибки
    error_data[1] = error_code;
    // Байты 2-7: доп. информация об ошибке
    
    send_can_message(CAN_STATUS_ID, error_data, 8);
    
    my_printf("[CAN] Error sent: code=0x%02X\n", error_code);
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
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
        my_printf("[CAN] ERROR: Failed to send message ID 0x%03X\n", id);
    }
}

// ============================================
// УСТАНОВКА ФИКСИРОВАННОЙ ЧАСТОТЫ НА ТАЙМЕРЫ
// ============================================

void set_fixed_frequency_to_timers(uint32_t freq_hz, uint16_t psc, uint16_t arr, uint8_t channel_mask)
{
    my_printf("[CAN] Setting fixed freq: %lu Hz, PSC=%u, ARR=%u to channels: 0x%02X\n", 
              freq_hz, psc, arr, channel_mask);
    
    // Рассчитываем фактическую частоту
    uint32_t actual_freq = APB1_CLK / ((psc + 1) * (arr + 1));
    int32_t error = (int32_t)(actual_freq - freq_hz);
    float error_percent = (float)error * 100.0f / freq_hz;
    
    my_printf("[CAN] Actual frequency: %lu Hz, Error: %ld Hz (%.3f%%)\n", 
              actual_freq, error, error_percent);
    
    // Устанавливаем на активные таймеры
    for(int i = 0; i < 4; i++) {
        if(channel_mask & (1 << i)) {
            switch(i) {
                case 0: // TIM1
                    TIM1->CR1 &= ~TIM_CR1_CEN;
                    TIM1->PSC = psc;
                    TIM1->ARR = arr;
                    TIM1->CNT = 0;
                    TIM1->CR1 |= TIM_CR1_CEN;
                    break;
                case 1: // TIM2
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    TIM2->PSC = psc;
                    TIM2->ARR = arr;
                    TIM2->CNT = 0;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    break;
                case 2: // TIM3
                    TIM3->CR1 &= ~TIM_CR1_CEN;
                    TIM3->PSC = psc;
                    TIM3->ARR = arr;
                    TIM3->CNT = 0;
                    TIM3->CR1 |= TIM_CR1_CEN;
                    break;
                case 3: // TIM4
                    TIM4->CR1 &= ~TIM_CR1_CEN;
                    TIM4->PSC = psc;
                    TIM4->ARR = arr;
                    TIM4->CNT = 0;
                    TIM4->CR1 |= TIM_CR1_CEN;
                    break;
            }
            my_printf("[CAN] Timer %d set: PSC=%u, ARR=%u, Freq=%lu Hz\n", 
                      i+1, psc, arr, actual_freq);
        }
    }
}


// Функция для точного расчета PSC и ARR
void calculate_optimal_psc_arr(uint32_t freq_hz, uint16_t *psc, uint16_t *arr)
{
    uint32_t best_error = 0xFFFFFFFF;
    uint16_t best_psc = 0;
    uint16_t best_arr = 0;
    
    if(freq_hz == 0) {
        *psc = 0;
        *arr = 65535;
        return;
    }
    
    // Максимальный делитель
    uint32_t max_div = APB1_CLK / freq_hz;
    
    // Если делитель меньше 65536, используем PSC=0
    if(max_div <= 65536) {
        *psc = 0;
        *arr = max_div - 1;
        return;
    }
    
    // Иначе ищем оптимальные PSC и ARR
    for(uint16_t p = 1; p < 1000; p++) {  // Ограничим PSC до 1000 для скорости
        uint32_t arr_candidate = max_div / (p + 1) - 1;
        
        if(arr_candidate > 65535) continue;
        if(arr_candidate < 2) break;  // Слишком маленький ARR
        
        uint32_t actual_freq = APB1_CLK / ((p + 1) * (arr_candidate + 1));
        uint32_t error = (actual_freq > freq_hz) ? (actual_freq - freq_hz) : (freq_hz - actual_freq);
        
        if(error < best_error) {
            best_error = error;
            best_psc = p;
            best_arr = (uint16_t)arr_candidate;
        }
        
        // Если нашли идеальное совпадение, выходим
        if(error == 0) break;
    }
    
    // Если не нашли, используем последние значения
    if(best_psc == 0 && best_arr == 0) {
        best_psc = (uint16_t)(max_div / 65536);
        best_arr = 65535;
    }
    
    *psc = best_psc;
    *arr = best_arr;
    
    // Проверка расчета
    uint32_t actual = APB1_CLK / ((*psc + 1) * (*arr + 1));
    uint32_t error = (actual > freq_hz) ? (actual - freq_hz) : (freq_hz - actual);
    float error_percent = (float)error * 100.0f / freq_hz;
    
    my_printf("[CALC] Target: %lu Hz, PSC=%u, ARR=%u, Actual: %lu Hz, Error: %lu Hz (%.3f%%)\n",
              freq_hz, *psc, *arr, actual, error, error_percent);
}
