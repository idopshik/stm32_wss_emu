/**
 * can_handler.c - VERSION 4.5 (DDS INTEGRATION)
 * 
 * Изменения v4.5:
 * - RPM режим использует wss_dds_set_rpm() вместо set_new_speeds()
 * - set_new_speeds() больше не используется (старый подход с таймерами)
 */

#include "can_handler.h"
#include "can_commands.h"
#include "system_modes.h"
#include "wss_dds.h"        // ← DDS для RPM режима
#include <string.h>
#include <stdio.h>

FDCAN_RxHeaderTypeDef RxHeader1;
extern FDCAN_HandleTypeDef hfdcan1;
extern void my_printf(const char *fmt, ...);

uint8_t canRX[8] = {0};
uint8_t newRPMmessage = 0;
uint8_t freshCanCmd = 0;
uint8_t canCmdData[8] = {0};

void can_handler_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};
    
    // Настройка фильтра 0: ID 0x003
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x003;  // ID для приема
    sFilterConfig.FilterID2    = 0x07FF;  // Маска (все биты)
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // Настройка фильтра 1: ID 0x004
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterID1    = 0x004;  // ID для приема
    sFilterConfig.FilterID2    = 0x7FF;  // Маска
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // Глобальный фильтр
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, 
        FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    // ТОЛЬКО RX прерывания
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    my_printf("[CAN] Initialized\n");
}

void can_process_in_main(void)
{
    if (newRPMmessage == 1) {
        newRPMmessage = 0;
        
        // DBC: Wheel_speed ID=0x003, big-endian, 16-bit, factor=0.02 RPM
        uint16_t raw[4];
        raw[0] = ((uint16_t)canRX[0] << 8) | canRX[1];  // FL
        raw[1] = ((uint16_t)canRX[2] << 8) | canRX[3];  // FR
        raw[2] = ((uint16_t)canRX[4] << 8) | canRX[5];  // RL
        raw[3] = ((uint16_t)canRX[6] << 8) | canRX[7];  // RR
        
        // ✅ RPM режим: используем DDS вместо старого подхода с таймерами
        wss_dds_set_rpm(raw);
        
        // Обновляем флаг и время для LED индикации
        g_system_state.rpm_signal_active = 1;
        g_system_state.led_last_toggle_time = HAL_GetTick();
    }
    
    if (freshCanCmd == 1) {
        freshCanCmd = 0;
        process_can_command(canCmdData);
    }
    
    if (can_tx_status_pending) {
        can_tx_status_pending = 0;
        send_system_status();
    }
    
    if (can_tx_error_pending) {
        can_tx_error_pending = 0;
        send_error_response(can_tx_error_code);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // КРИТИЧЕСКИ ВАЖНО: очистить флаг ПРЕЖДЕ всего
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE);
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        
        // Проверить уровень FIFO
        uint8_t fifoLevel = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
        
        // Прочитать все сообщения из FIFO
        for (uint8_t i = 0; i < fifoLevel; i++) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
                break;
            }
            
            if (RxHeader1.Identifier == 0x003) {
                newRPMmessage = 1;
            }
            else if (RxHeader1.Identifier == 0x004) {
                memcpy(canCmdData, canRX, 8);
                freshCanCmd = 1;
            }
        }
    }
}
