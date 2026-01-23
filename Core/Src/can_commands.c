#include "can_commands.h"
#include "system_modes.h"
#include <string.h>

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
            
            my_printf("[CAN] Command: SET FIXED FREQ = %lu Hz, PSC = %u\n", 
                      freq_hz, psc_value);
            
            g_system_state.target_frequency_hz = freq_hz;
            
            if(channel_mask == 0xFF) {
                set_all_channels_active(1);
            }
            
            // TODO: Здесь позже установим частоту на таймеры
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
        
        case CMD_SAVE_TO_EEPROM: {
            my_printf("[CAN] Command: SAVE TO EEPROM\n");
            // TODO: Реализуем позже
            my_printf("[CAN] EEPROM save not implemented yet\n");
            g_system_state.eeprom_saved = 0;
            break;
        }
        
        case CMD_FACTORY_RESET: {
            my_printf("[CAN] Command: FACTORY RESET\n");
            // TODO: Реализуем позже
            my_printf("[CAN] Factory reset not implemented yet\n");
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

void send_system_status(void)
{
    uint8_t status_data[8] = {0};
    
    // Байт 0: Текущий режим
    status_data[0] = (uint8_t)g_system_state.current_mode;
    
    // Байт 1: Маска активных каналов
    status_data[1] = g_system_state.channel_mask;
    
    // Байты 2-3: Частота TIM1 / 10 Гц (uint16_t)
    if(g_system_state.target_frequency_hz > 0) {
        uint16_t freq_x10 = (uint16_t)(g_system_state.target_frequency_hz / 10);
        status_data[2] = freq_x10 & 0xFF;
        status_data[3] = (freq_x10 >> 8) & 0xFF;
    }
    
    // Байты 4-5: Частота TIM2 / 10 Гц (пока такая же)
    status_data[4] = status_data[2];
    status_data[5] = status_data[3];
    
    // Байты 6-7: Доп. информация
    status_data[6] = g_system_state.pwm_duty_percent;
    status_data[7] = g_system_state.analog_signal_present;
    
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
