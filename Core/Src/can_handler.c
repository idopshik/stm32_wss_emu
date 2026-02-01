// can_handler.c
#include "can_handler.h"
#include "wheel_control.h"
#include "system_modes.h"
#include "can_commands.h"
#include <string.h>  // для memcpy
#include <stdio.h>   //для my_printf

extern FDCAN_HandleTypeDef hfdcan1;      // Из main.c
extern FDCAN_RxHeaderTypeDef RxHeader1;  // Из main.c
extern void my_printf(const char *fmt, ...);  // Из main.c

// Константы
#define CAN_SPECIAL_ID 0x003  // ← ДОБАВИТЬ если нет в can_handler.h

uint8_t canRX[8] = {0};
uint8_t freshCanMsg = 0;
uint8_t freshCanCmd = 0;
uint8_t canCmdData[8] = {0};

void can_handler_init(void)
{
    // Настройка фильтров
    FDCAN_FilterTypeDef sFilterConfig;

    // Фильтр 0: Принимаем ID 0x003 (RPM данные)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;  // ID 0x003
    sFilterConfig.FilterID2 = 0x07FF;          // Маска
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // Фильтр 1: Принимаем ID 0x004 (команды управления)
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = CAN_CMD_ID;      // ID 0x004
    sFilterConfig.FilterID2 = 0x07FF;          // Маска
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Глобальный фильтр
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_REJECT,          // Стандартные ID не прошедшие фильтр
        FDCAN_REJECT,          // Расширенные ID не прошедшие фильтр
        FDCAN_FILTER_REMOTE,   // Удаленные фреймы с фильтром
        FDCAN_FILTER_REMOTE)   // Удаленные фреймы без фильтра
        != HAL_OK) {
        Error_Handler();
    }

    // Активируем прерывание
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    
    // Запускаем FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    my_printf("CAN Filter: ID 0x003 & 0x004\n");

}

void can_process_in_main(void)
{
    // Обработка RPM данных (ID 0x003)
    if (freshCanMsg == 1) {
        freshCanMsg = 0;
        
        // Используем правильный тип из system_modes.h
        if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
            int vFLrpm = (uint8_t)canRX[0] << 8 | (uint8_t)canRX[1];
            int vFRrpm = (uint8_t)canRX[2] << 8 | (uint8_t)canRX[3];
            int vRLrpm = (uint8_t)canRX[4] << 8 | (uint8_t)canRX[5];
            int vRRrpm = (uint8_t)canRX[6] << 8 | (uint8_t)canRX[7];
            
            // Нужно передать whl_arr из main.c
            // Либо сделать его глобальным, либо передавать как параметр
            set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);  // ← 5 параметров
            
            g_system_state.rpm_mode_active = 1;
            g_system_state.last_can_command_time = HAL_GetTick();
        }
        
        // Индикация приема
        g_system_state.can_activity_flag = 1;
    }
    
    // Обработка команд (ID 0x004)
    if (freshCanCmd == 1) {
        freshCanCmd = 0;
        process_can_command(canCmdData);  // ← Вызываем из can_commands.c
    }
    
    // Обработка статусов и ошибок
    if (can_tx_status_pending) {
        send_system_status();
        can_tx_status_pending = 0;
    }
    
    if (can_tx_error_pending) {
        send_error_response(can_tx_error_code);
        can_tx_error_pending = 0;
    }
}


// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            Error_Handler();
        }
        else {
            if (RxHeader1.Identifier == CAN_SPECIAL_ID) {
                // RPM данные
                freshCanMsg = 1;
            }
            else if (RxHeader1.Identifier == CAN_CMD_ID) {  // 0x004
                // Команды управления
                memcpy(canCmdData, canRX, 8);
                freshCanCmd = 1;  // ТОЛЬКО флаг, обработка в основном цикле
                // НЕ вызывать process_can_command здесь!
            }
            else {
                my_printf("wrongID: %#x \n\r", RxHeader1.Identifier);
            }
        }
    }
}


