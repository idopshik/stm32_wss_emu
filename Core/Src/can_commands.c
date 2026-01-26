/**
 * can_commands.c - VERSION 2
 * 
 * Добавлено:
 * - Hi-Z режим (команда 0x06)
 * - Улучшенный статус с uptime и флагами
 */

#include "can_commands.h"
#include "system_modes.h"
#include <string.h>
#include <stdio.h>

#ifndef APB1_CLK
#define APB1_CLK 150000000
#endif

extern void my_printf(const char *fmt, ...);
extern FDCAN_HandleTypeDef hfdcan1;

// Прототипы локальных функций
void send_can_message(uint32_t id, uint8_t* data, uint8_t length);

// ============================================
// ОБРАБОТКА КОМАНД - С HI-Z
// ============================================

void process_can_command(uint8_t* data)
{
    if(data == NULL) return;
    
    uint8_t command = data[0];
    
    #if DEBUG_CAN_COMMANDS
    my_printf("[CAN] Cmd: 0x%02X\n", command);
    #endif
    
    switch(command) {
        case CMD_DISABLE_OUTPUT: {
            uint8_t channel_mask = data[1];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Disable, mask: 0x%02X\n", channel_mask);
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
        
        case CMD_SET_RPM_MODE: {
            uint8_t channel_mask = data[1];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] RPM mode, mask: 0x%02X\n", channel_mask);
            #endif
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            system_switch_mode(MODE_RPM_DYNAMIC);
            break;
        }
        
        case CMD_SET_FIXED_FREQ: {
            uint8_t channel_mask = data[1];
            
            // Байты 2-5: частота в Гц (little-endian)
            uint32_t freq_hz = ((uint32_t)data[5] << 24) |
                               ((uint32_t)data[4] << 16) |
                               ((uint32_t)data[3] << 8) |
                               data[2];
            
            // Байты 6-7: PSC значение (0xFFFF = авто)
            uint16_t psc_value = ((uint16_t)data[7] << 8) | data[6];
            uint16_t arr_value;
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Fixed freq: %lu Hz, PSC: %u\n", freq_hz, psc_value);
            #endif
            
            g_system_state.target_frequency_hz = freq_hz;
            
            if(psc_value == 0xFFFF) {
                calculate_optimal_psc_arr(freq_hz, &psc_value, &arr_value);
            } else {
                arr_value = (APB1_CLK / (freq_hz * (psc_value + 1))) - 1;
                if(arr_value > 65535) arr_value = 65535;
                if(arr_value < 1) arr_value = 1;
            }
            
            set_fixed_frequency_to_timers(freq_hz, psc_value, arr_value, channel_mask);
            
            for(int i = 0; i < 4; i++) {
                if(channel_mask & (1 << i)) {
                    g_system_state.psc_values[i] = psc_value;
                    g_system_state.arr_values[i] = arr_value;
                }
            }
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            system_switch_mode(MODE_FIXED_FREQUENCY);
            break;
        }
        
        case CMD_SET_PWM_MODE: {
            uint8_t channel_mask = data[1];
            
            uint32_t freq_hz = ((uint32_t)data[5] << 24) |
                               ((uint32_t)data[4] << 16) |
                               ((uint32_t)data[3] << 8) |
                               data[2];
            
            uint16_t duty = ((uint16_t)data[7] << 8) | data[6];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] PWM: %lu Hz, Duty: %u%%\n", freq_hz, duty);
            #endif
            
            if(duty > 100) duty = 100;
            
            g_system_state.target_frequency_hz = freq_hz;
            g_system_state.pwm_duty_percent = (uint8_t)duty;
            
            if(freq_hz > 0) {
                uint16_t psc, arr;
                calculate_optimal_psc_arr(freq_hz, &psc, &arr);
                
                TIM2->CR1 &= ~TIM_CR1_CEN;
                TIM2->PSC = psc;
                TIM2->ARR = arr;
                TIM2->CCR1 = (arr * duty) / 100;
                TIM2->CNT = 0;
                TIM2->CR1 |= TIM_CR1_CEN;
            }
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            
            system_switch_mode(MODE_PWM);
            break;
        }
        
        case CMD_SET_ANALOG_FOLLOW: {
            uint8_t channel_mask = data[1];
            
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Analog follow mode\n");
            #endif
            
            if(g_system_state.hi_impedance_active) {
                exit_hi_impedance_mode();
            }
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            system_switch_mode(MODE_ANALOG_FOLLOW);
            break;
        }
        
        case CMD_SET_HI_IMPEDANCE: {
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Hi-Z mode request\n");
            #endif
            
            // Только флаг - реальная обработка будет в main loop
            can_tx_status_pending = 1;  // Для немедленного ответа
            // Само переключение в Hi-Z сделаем в main loop
            // чтобы не блокировать прерывание
            g_system_state.pending_hi_z = 1;
            break;
        }
        
        case CMD_REQUEST_STATUS: {
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Status request\n");
            #endif
            
            // ТОЛЬКО ФЛАГ, НЕ ОТПРАВЛЯЕМ ЗДЕСЬ!
            can_tx_status_pending = 1;
            break;
        }
        
        default: {
            #if DEBUG_CAN_COMMANDS
            my_printf("[CAN] Unknown command: 0x%02X\n", command);
            #endif
            
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
    my_printf("\n[HI-Z] === ENTERING HI-IMPEDANCE MODE ===\n");
    
    // Останавливаем все таймеры
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    my_printf("[HI-Z] All timers stopped\n");
    
    // Переводим GPIO в режим INPUT (Hi-Z)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // PA8 (TIM1_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA8 → INPUT (Hi-Z)\n");
    
    // PA5 (TIM2_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA5 → INPUT (Hi-Z)\n");
    
    // PA6 (TIM3_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA6 → INPUT (Hi-Z)\n");
    
    // PB6 (TIM4_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    my_printf("[HI-Z] PB6 → INPUT (Hi-Z)\n");
    
    // Устанавливаем флаг и режим
    g_system_state.hi_impedance_active = 1;
    system_switch_mode(MODE_HI_IMPEDANCE);
    
    my_printf("[HI-Z] Mode active - safe for external signals\n");
    my_printf("[HI-Z] =======================================\n\n");
}

// ============================================
// HI-Z РЕЖИМ - ВЫХОД
// ============================================

void exit_hi_impedance_mode(void)
{
    my_printf("\n[HI-Z] === EXITING HI-IMPEDANCE MODE ===\n");
    
    // Восстанавливаем GPIO как Alternate Function для таймеров
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // Для STM32G431 нужно использовать правильные AF значения:
    // PA8 (TIM1_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;  // Исправлено с AF6 на AF2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA8 → AF2 (TIM1_CH1)\n");  // Исправь комментарий тоже!
    
    // PA5 (TIM2_CH1) - AF1
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA5 → AF1 (TIM2_CH1)\n");
    
    // PA6 (TIM3_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    my_printf("[HI-Z] PA6 → AF2 (TIM3_CH1)\n");
    
    // PB6 (TIM4_CH1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    my_printf("[HI-Z] PB6 → AF2 (TIM4_CH1)\n");
    
    // ВОССТАНАВЛИВАЕМ ТАЙМЕРЫ!
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        // Восстанавливаем частоту
        for(int i = 0; i < 4; i++) {
            if(g_system_state.channel_mask & (1 << i)) {
                // Получаем сохраненные значения
                uint16_t psc = g_system_state.psc_values[i];
                uint16_t arr = g_system_state.arr_values[i];
                
                // Применяем к соответствующему таймеру
                TIM_TypeDef* TIMx = NULL;
                switch(i) {
                    case 0: TIMx = TIM1; break;
                    case 1: TIMx = TIM2; break;
                    case 2: TIMx = TIM3; break;
                    case 3: TIMx = TIM4; break;
                }
                
                if(TIMx != NULL) {
                    TIMx->CR1 &= ~TIM_CR1_CEN;
                    TIMx->PSC = psc;
                    TIMx->ARR = arr;
                    TIMx->CNT = 0;
                    TIMx->CR1 |= TIM_CR1_CEN;
                    
                    my_printf("[HI-Z] TIM%d restored: PSC=%u, ARR=%u\n", 
                              i+1, psc, arr);
                }
            }
        }
    } else if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        // Перезапускаем таймеры
        TIM1->CR1 |= TIM_CR1_CEN;
        TIM2->CR1 |= TIM_CR1_CEN;
        TIM3->CR1 |= TIM_CR1_CEN;
        TIM4->CR1 |= TIM_CR1_CEN;
        my_printf("[HI-Z] RPM mode: all timers restarted\n");
    }
    
    g_system_state.hi_impedance_active = 0;
    
    my_printf("[HI-Z] GPIO restored to timer outputs\n");
    my_printf("[HI-Z] ===================================\n\n");
}


uint32_t get_timer_output_frequency(TIM_TypeDef* TIMx)
{
    if(TIMx == NULL || (TIMx->CR1 & TIM_CR1_CEN) == 0) {
        return 0;
    }
    
    uint32_t psc = TIMx->PSC;
    uint32_t arr = TIMx->ARR;
    
    // Защита от некорректных значений
    if(arr == 0 || arr == 0xFFFF) return 0;
    
    // Формула: F_timer = APB1_CLK / ((PSC+1) × (ARR+1))
    uint64_t divider = (uint64_t)(psc + 1) * (arr + 1);
    if(divider == 0) return 0;
    
    uint32_t timer_freq = (uint32_t)(APB1_CLK / divider);
    
    // КОРРЕКЦИЯ ДЛЯ РЕЖИМА TOGGLE (TIM2)
    // TIM2 настроен в режиме TIM_OCMODE_TOGGLE - частота на выходе в 2 раза меньше
    if(TIMx == TIM2) {
        // Проверяем, что канал 1 действительно в режиме TOGGLE и включен
        uint32_t ccmr1 = TIMx->CCMR1;
        uint32_t ccer = TIMx->CCER;
        
        // Режим TOGGLE = биты OC1M[2:0] = 001
        uint32_t oc1m = (ccmr1 & TIM_CCMR1_OC1M) >> TIM_CCMR1_OC1M_Pos;
        
        if(oc1m == TIM_OCMODE_TOGGLE && (ccer & TIM_CCER_CC1E)) {
            // TOGGLE режим: выходная частота = частота таймера / 2
            timer_freq /= 2;
        }
    }
    // TIM1, TIM3, TIM4: режим сравнения не используется, полная частота
    
    return timer_freq;
}

// ============================================
// ОТПРАВКА СТАТУСА (ID 0x006) - ИСПРАВЛЕННЫЙ
// ============================================

void send_system_status(void)
{
    uint8_t status_data[8] = {0};
    
    // Byte 0: Текущий режим
    status_data[0] = (uint8_t)g_system_state.current_mode;
    
    // Byte 1: Маска активных каналов
    status_data[1] = g_system_state.channel_mask;
    
    // Bytes 2-3: Частота (зависит от режима)
    if(g_system_state.current_mode == MODE_FIXED_FREQUENCY || 
       g_system_state.current_mode == MODE_PWM) {
        // Fixed/PWM режим: целевая частота ×10 (0.1 Гц)
        uint16_t freq_x10 = (uint16_t)(g_system_state.target_frequency_hz * 10);
        if(freq_x10 > 65535) freq_x10 = 65535;
        status_data[2] = freq_x10 & 0xFF;
        status_data[3] = (freq_x10 >> 8) & 0xFF;
    }
    else if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        // RPM режим: МАКСИМАЛЬНАЯ ВЫХОДНАЯ частота из активных каналов ×10 (0.1 Гц)
        uint32_t max_freq = 0;
        TIM_TypeDef* timers[4] = {TIM1, TIM2, TIM3, TIM4};
        uint8_t any_active = 0;
        
        for(int i = 0; i < 4; i++) {
            if(g_system_state.channel_mask & (1 << i)) {
                uint32_t freq = get_timer_output_frequency(timers[i]);
                if(freq > 0) {
                    any_active = 1;
                    if(freq > max_freq) {
                        max_freq = freq;
                    }
                }
            }
        }
        
        uint16_t max_freq_x10 = 0;
        if(max_freq > 0) {
            if(max_freq > 6553) {  // 6553.5 Гц максимум (65535 / 10)
                max_freq_x10 = 65535;
            } else {
                max_freq_x10 = (uint16_t)(max_freq * 10);
            }
        }
        
        status_data[2] = max_freq_x10 & 0xFF;
        status_data[3] = (max_freq_x10 >> 8) & 0xFF;
    }
    else {
        // Другие режимы (ANALOG_FOLLOW, HI_IMPEDANCE, etc.)
        status_data[2] = 0;
        status_data[3] = 0;
    }
    
    // Byte 4: Зависит от режима
    if(g_system_state.current_mode == MODE_PWM) {
        // PWM режим: скважность
        status_data[4] = g_system_state.pwm_duty_percent;
    }
    else if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        // RPM режим: состояние колёс (маска + флаги)
        status_data[4] = g_system_state.channel_mask;  // Биты 0-3: активные каналы
        
        // Добавляем флаги в биты 4-7
        uint8_t wheel_flags = 0;
        
        // Проверяем есть ли активные таймеры с выходной частотой > 0
        TIM_TypeDef* timers[4] = {TIM1, TIM2, TIM3, TIM4};
        uint8_t any_active_freq = 0;
        uint8_t all_running = 1;
        uint8_t has_errors = 0;
        
        for(int i = 0; i < 4; i++) {
            if(g_system_state.channel_mask & (1 << i)) {
                uint32_t freq = get_timer_output_frequency(timers[i]);
                
                if(freq > 0) {
                    any_active_freq = 1;
                }
                
                if((timers[i]->CR1 & TIM_CR1_CEN) == 0) {
                    all_running = 0;
                }
                
                if(timers[i]->ARR == 0 || timers[i]->ARR > 65000) {
                    has_errors = 1;
                }
            }
        }
        
        if(any_active_freq) {
            wheel_flags |= (1 << 4);  // Bit 4: есть активные таймеры с выходной частотой > 0
        }
        
        if(all_running && g_system_state.channel_mask != 0) {
            wheel_flags |= (1 << 5);  // Bit 5: все активные таймеры запущены
        }
        
        if(has_errors) {
            wheel_flags |= (1 << 6);  // Bit 6: есть ошибки конфигурации
        }
        
        // Bit 7: зарезервировано = 0
        
        // Объединяем маску каналов и флаги
        status_data[4] |= (wheel_flags << 4);
    }
    else {
        // Другие режимы: 0
        status_data[4] = 0;
    }
    
    // Byte 5: Флаги состояния
    status_data[5] = 0;
    if(g_system_state.analog_signal_present) {
        status_data[5] |= 0x01;  // Bit 0: analog signal
    }
    if(g_system_state.hi_impedance_active) {
        status_data[5] |= 0x02;  // Bit 1: Hi-Z mode
    }
    
    // Bytes 6-7: Uptime в секундах
    uint32_t uptime = system_get_uptime_seconds();
    if(uptime > 65535) uptime = 65535;
    status_data[6] = uptime & 0xFF;
    status_data[7] = (uptime >> 8) & 0xFF;
    
    // Отправляем CAN сообщение
    send_can_message(CAN_STATUS_ID, status_data, 8);
    
    #if DEBUG_CAN_TX
    my_printf("[STATUS] Sent: mode=0x%02X (%s), ch=0x%02X\n",
              status_data[0], 
              get_mode_name(g_system_state.current_mode),
              status_data[1]);
    
    if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
        uint16_t freq = (status_data[3] << 8) | status_data[2];
        uint8_t wheel_byte = status_data[4];
        my_printf("[STATUS] Max OUTPUT freq: %.1f Hz\n", freq / 10.0f);
        my_printf("[STATUS] Wheel state: mask=0x%02X, flags=0x%02X\n",
                  wheel_byte & 0x0F, (wheel_byte >> 4) & 0x0F);
        
        // Детальный дебаг
        TIM_TypeDef* timers[4] = {TIM1, TIM2, TIM3, TIM4};
        for(int i = 0; i < 4; i++) {
            if(g_system_state.channel_mask & (1 << i)) {
                uint32_t out_freq = get_timer_output_frequency(timers[i]);
                uint32_t timer_freq = APB1_CLK / ((timers[i]->PSC + 1) * (timers[i]->ARR + 1));
                my_printf("[STATUS] TIM%d: output=%.1f Hz, timer=%.1f Hz, CEN=%d\n",
                         i+1, out_freq / 1.0f, timer_freq / 1.0f,
                         (timers[i]->CR1 & TIM_CR1_CEN) ? 1 : 0);
            }
        }
    }
    else if(g_system_state.current_mode == MODE_FIXED_FREQUENCY || 
              g_system_state.current_mode == MODE_PWM) {
        uint16_t freq = (status_data[3] << 8) | status_data[2];
        my_printf("[STATUS] Target freq: %.1f Hz\n", freq / 10.0f);
    }
    
    my_printf("[STATUS] Flags: analog=%d, hi_z=%d, uptime=%lu sec\n",
              g_system_state.analog_signal_present,
              g_system_state.hi_impedance_active,
              uptime);
    #endif
}

// ============================================
// ОТПРАВКА ОШИБКИ
// ============================================

void send_error_response(uint8_t error_code)
{
    uint8_t error_data[8] = {0};
    error_data[0] = 0xFF;  // Флаг ошибки
    error_data[1] = error_code;
    
    send_can_message(CAN_STATUS_ID, error_data, 8);
    
    #if DEBUG_CAN_ERROR
    my_printf("[ERROR] Code: 0x%02X\n", error_code);
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
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;  // ← ИЗМЕНИЛ НА PASSIVE
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}

// ============================================
// УСТАНОВКА ФИКСИРОВАННОЙ ЧАСТОТЫ
// ============================================

void set_fixed_frequency_to_timers(uint32_t freq_hz, uint16_t psc, uint16_t arr, uint8_t channel_mask)
{
    uint32_t actual_freq = APB1_CLK / ((psc + 1) * (arr + 1));
    int32_t error = (int32_t)(actual_freq - freq_hz);
    float error_percent = (float)error * 100.0f / freq_hz;
    
    my_printf("[FIXED_FREQ] PSC=%u, ARR=%u\n", psc, arr);
    my_printf("[FIXED_FREQ] Actual: %lu Hz, Error: %ld Hz (%.4f%%)\n", 
              actual_freq, error, error_percent);
    
    for(int i = 0; i < 4; i++) {
        if(channel_mask & (1 << i)) {
            TIM_TypeDef* TIMx = NULL;
            switch(i) {
                case 0: TIMx = TIM1; break;
                case 1: TIMx = TIM2; break;
                case 2: TIMx = TIM3; break;
                case 3: TIMx = TIM4; break;
            }
            
            if(TIMx != NULL) {
                TIMx->CR1 &= ~TIM_CR1_CEN;
                TIMx->PSC = psc;
                TIMx->ARR = arr;
                TIMx->CNT = 0;
                TIMx->CR1 |= TIM_CR1_CEN;
                
                my_printf("[FIXED_FREQ] TIM%d: %lu Hz\n", i+1, actual_freq);
            }
        }
    }
    
    my_printf("\n");
}

// ============================================
// РАСЧЁТ PSC/ARR
// ============================================

void calculate_optimal_psc_arr(uint32_t freq_hz, uint16_t *psc, uint16_t *arr)
{
    if(freq_hz == 0 || freq_hz > APB1_CLK) {
        *psc = 0;
        *arr = 65535;
        my_printf("[ERROR] Invalid frequency: %lu Hz\n", freq_hz);
        return;
    }
    
    uint32_t total_div = APB1_CLK / freq_hz;
    
    if(total_div <= 65536) {
        *psc = 0;
        *arr = total_div - 1;
        
        uint32_t actual = APB1_CLK / (*arr + 1);
        float error_pct = ((float)actual - freq_hz) * 100.0f / freq_hz;
        
        my_printf("[CALC] Simple: PSC=0, ARR=%u, Actual=%lu Hz, Err=%.4f%%\n",
                  *arr, actual, error_pct);
        return;
    }
    
    uint32_t best_error = 0xFFFFFFFF;
    uint16_t best_psc = 0;
    uint16_t best_arr = 0;
    
    uint16_t psc_start = (uint16_t)(total_div / 65536);
    if(psc_start == 0) psc_start = 1;
    
    for(uint16_t p = psc_start; p <= 5000 && p < 65536; p++) {
        uint32_t arr_calc = (total_div / (p + 1)) - 1;
        
        if(arr_calc > 65535) continue;
        if(arr_calc < 2) break;
        
        uint32_t actual_freq = APB1_CLK / ((p + 1) * (arr_calc + 1));
        uint32_t error = (actual_freq > freq_hz) ? 
                         (actual_freq - freq_hz) : (freq_hz - actual_freq);
        
        if(error < best_error) {
            best_error = error;
            best_psc = p;
            best_arr = (uint16_t)arr_calc;
        }
        
        if(error == 0) break;
    }
    
    if(best_psc == 0 && best_arr == 0) {
        best_psc = (uint16_t)(total_div / 65536);
        best_arr = 65535;
    }
    
    *psc = best_psc;
    *arr = best_arr;
    
    uint32_t actual = APB1_CLK / ((*psc + 1) * (*arr + 1));
    float error_pct = ((float)actual - freq_hz) * 100.0f / freq_hz;
    
    my_printf("[CALC] Optimal: PSC=%u, ARR=%u\n", *psc, *arr);
    my_printf("[CALC] Target=%lu Hz, Actual=%lu Hz, Error=%.4f%%\n",
              freq_hz, actual, error_pct);
}
