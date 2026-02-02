#include "can_handler.h"
#include "can_commands.h"
#include "system_modes.h"
#include "wheel_control.h"
#include <string.h>
#include <stdio.h>

/* extern FDCAN_RxHeaderTypeDef RxHeader1; */
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
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x003;  // ID для приема
    sFilterConfig.FilterID2 = 0x7FF;  // Маска (все биты)
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // Настройка фильтра 1: ID 0x004
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = 0x004;  // ID для приема
    sFilterConfig.FilterID2 = 0x7FF;  // Маска
    
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
    
        
        int vFLrpm = ((uint16_t)canRX[0] << 8) | (uint16_t)canRX[1];
        int vFRrpm = ((uint16_t)canRX[2] << 8) | (uint16_t)canRX[3];
        int vRLrpm = ((uint16_t)canRX[4] << 8) | (uint16_t)canRX[5];
        int vRRrpm = ((uint16_t)canRX[6] << 8) | (uint16_t)canRX[7];
        
        
        set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm);

        g_system_state.rpm_mode_active = 1;  // ← Флаг что данные приходят
        g_system_state.led_last_toggle_time = HAL_GetTick();
        
}
    
    if (freshCanCmd == 1) {
        freshCanCmd = 0;
        my_printf("[BEFORE CMD] can_tx_status_pending = %d\n", can_tx_status_pending);
        process_can_command(canCmdData);
        my_printf("[AFTER CMD] can_tx_status_pending = %d\n", can_tx_status_pending);
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
        my_printf("[CAN RX] Callback triggered\n");
        
        // Проверить уровень FIFO
        uint8_t fifoLevel = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
        my_printf("[CAN RX] FIFO level: %u\n", fifoLevel);
        
        // Прочитать все сообщения из FIFO
        for (uint8_t i = 0; i < fifoLevel; i++) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
                my_printf("[CAN RX] Error reading message\n");
                break;
            }
            
            my_printf("[CAN RX] ID: 0x%03lX\n", RxHeader1.Identifier);
            
            if (RxHeader1.Identifier == CAN_SPECIAL_ID) {
                newRPMmessage = 1;
                my_printf("[CAN RX] RPM message flagged\n");
            }
            else if (RxHeader1.Identifier == CAN_CMD_ID) {
                memcpy(canCmdData, canRX, 8);
                freshCanCmd = 1;
                my_printf("[CAN RX] Command flagged: 0x%02X\n", canCmdData[0]);
            }
        }
    }
}
