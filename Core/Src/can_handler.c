/**
 * can_handler.c - ОБЪЕДИНЕННАЯ ВЕРСИЯ
 * 
 * Объединяет лучшие черты обоих вариантов:
 * - Парсинг Big-Endian как у Haiku
 * - Диагностика с возможностью включения
 * - Явное объявление whl_arr
 */

#include "can_handler.h"
#include "can_commands.h"
#include "system_modes.h"
#include "wheel_control.h"
#include <string.h>
#include <stdio.h>

uint32_t can_callback_count = 0;  // Счётчик вызовов callback
// ========== ДИАГНОСТИКА ==========
// Раскомментировать для отладки:
// #define DEBUG_CAN
// #define DEBUG_CAN_DETAIL

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef RxHeader1;
extern void my_printf(const char *fmt, ...);
extern whl_chnl *whl_arr[4];  // ← ВАЖНО: объявляем extern

// ========== БУФЕРЫ И ФЛАГИ ==========

uint8_t canRX[8] = {0};
uint8_t freshCanMsg = 0;      // Флаг: есть новое RPM сообщение (ID 0x003)
uint8_t freshCanCmd = 0;      // Флаг: есть новая команда (ID 0x004)
uint8_t canCmdData[8] = {0};

// ========== ИНИЦИАЛИЗАЦИЯ CAN ==========

void can_handler_init(void)
{
    my_printf("[CAN INIT] ========== START ==========\n");
    
    FDCAN_FilterTypeDef sFilterConfig;

    // Фильтр 0
    my_printf("[CAN INIT] Configuring filter 0 (ID 0x003)...\n");
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;  // 0x003
    sFilterConfig.FilterID2 = 0x07FF;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        my_printf("[CAN ERROR] Filter 0 config FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Filter 0 OK\n");
    
    // Фильтр 1
    my_printf("[CAN INIT] Configuring filter 1 (ID 0x004)...\n");
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = CAN_CMD_ID;  // 0x004
    sFilterConfig.FilterID2 = 0x07FF;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        my_printf("[CAN ERROR] Filter 1 config FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Filter 1 OK\n");

    // Глобальный фильтр
    my_printf("[CAN INIT] Setting global filter...\n");
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
        != HAL_OK) {
        my_printf("[CAN ERROR] Global filter FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Global filter OK\n");

    // Прерывание
    my_printf("[CAN INIT] Activating RX_FIFO0 notification...\n");
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        my_printf("[CAN ERROR] ActivateNotification FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Notification activated\n");
    
    // Запуск
    my_printf("[CAN INIT] Starting FDCAN...\n");
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        my_printf("[CAN ERROR] FDCAN Start FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] FDCAN started\n");
    
    my_printf("[CAN INIT] ========== COMPLETE ==========\n");
}

// ========== ОБРАБОТКА В MAIN ЦИКЛЕ ==========

void can_process_in_main(void)
{
    // ===== ОБРАБОТКА RPM ДАННЫХ (ID 0x003) =====
    if (freshCanMsg == 1) {
        freshCanMsg = 0;
        
#ifdef DEBUG_CAN_DETAIL
        my_printf("[CAN] RPM message received\n");
#endif
        
        if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
            // Парсим Big-Endian значения (как у Haiku - ПРАВИЛЬНО)
            int vFLrpm = ((uint16_t)canRX[0] << 8) | (uint16_t)canRX[1];
            int vFRrpm = ((uint16_t)canRX[2] << 8) | (uint16_t)canRX[3];
            int vRLrpm = ((uint16_t)canRX[4] << 8) | (uint16_t)canRX[5];
            int vRRrpm = ((uint16_t)canRX[6] << 8) | (uint16_t)canRX[7];
            
#ifdef DEBUG_CAN_DETAIL
            my_printf("[CAN] RPM: FL=%d, FR=%d, RL=%d, RR=%d\n",
                      vFLrpm, vFRrpm, vRLrpm, vRRrpm);
#endif
            
            // Передаем скорости в wheel_control
            set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);
            
            g_system_state.rpm_mode_active = 1;
            g_system_state.last_can_command_time = HAL_GetTick();
        }
        
        g_system_state.can_activity_flag = 1;
    }
    
    // ===== ОБРАБОТКА КОМАНД (ID 0x004) =====
    if (freshCanCmd == 1) {
        freshCanCmd = 0;
        
#ifdef DEBUG_CAN_DETAIL
        my_printf("[CAN] Command: 0x%02X\n", canCmdData[0]);
#endif
        
        process_can_command(canCmdData);
    }
    
    // ===== ОТПРАВКА ОЖИДАЮЩИХ СООБЩЕНИЙ =====
    
    // Статус
    if (can_tx_status_pending) {
        can_tx_status_pending = 0;
#ifdef DEBUG_CAN_DETAIL
        my_printf("[CAN] Sending status...\n");
#endif
        send_system_status();
    }
    
    // Ошибка
    if (can_tx_error_pending) {
        can_tx_error_pending = 0;
#ifdef DEBUG_CAN_DETAIL
        my_printf("[CAN] Sending error: 0x%02X\n", can_tx_error_code);
#endif
        send_error_response(can_tx_error_code);
    }
}

// ========== CALLBACK ПРЕРЫВАНИЯ ==========


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static uint32_t rx_count = 0;
    my_printf("[CALLBACK] Message #%lu\n", ++rx_count);
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        my_printf("[CALLBACK] RxFifo0ITs check passed\n");
        
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            my_printf("[CALLBACK ERROR] GetRxMessage failed!\n");
            Error_Handler();
        }
        
        my_printf("[CALLBACK] Got message ID: 0x%03lX\n", RxHeader1.Identifier);
        
        if (RxHeader1.Identifier == CAN_SPECIAL_ID) {
            freshCanMsg = 1;
            my_printf("[CALLBACK] → Set freshCanMsg = 1\n");
        }
        else if (RxHeader1.Identifier == CAN_CMD_ID) {
            memcpy(canCmdData, canRX, 8);
            freshCanCmd = 1;
            my_printf("[CALLBACK] → Set freshCanCmd = 1 (cmd=0x%02X)\n", canCmdData[0]);
        }
        else {
            my_printf("[CALLBACK WARN] Unknown ID: 0x%03lX\n", RxHeader1.Identifier);
        }
    }
}
