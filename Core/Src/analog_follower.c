/**
 * analog_follower.c
 * 
 * Аналоговый режим следования внешнему сигналу
 * - Input Capture на TIM8_CH1 (PC6)
 * - Slave Mode синхронизация TIM1/TIM3/TIM4
 * - Минимальный джиттер (<10нс)
 * - Отключение прерываний при >500 кГц
 */

#include "analog_follower.h"
#include "system_modes.h"
#include <stdio.h>

// Внешние переменные
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern void my_printf(const char *fmt, ...);

// Локальные переменные
static volatile uint32_t capture_count = 0;
static volatile uint32_t last_capture_time = 0;
static volatile uint32_t current_frequency = 0;
static volatile uint8_t signal_detected = 0;
static volatile uint8_t high_freq_mode = 0;  // >500 кГц

// ============================================
// ИНИЦИАЛИЗАЦИЯ АНАЛОГОВОГО РЕЖИМА
// ============================================

void analog_follower_init(void)
{
    // Пока пустая - будет заполнена при переходе в режим
    capture_count = 0;
    last_capture_time = 0;
    current_frequency = 0;
    signal_detected = 0;
    high_freq_mode = 0;
}

// ============================================
// ЗАПУСК АНАЛОГОВОГО РЕЖИМА
// ============================================

void analog_follower_start(void)
{
    my_printf("\n[ANALOG] Starting analog follower mode\n");
    my_printf("[ANALOG] Input: PC6 (TIM8_CH1)\n");
    my_printf("[ANALOG] Outputs: PA8(TIM1), PA6(TIM3), PB6(TIM4)\n");
    
    // Сбрасываем счётчики
    capture_count = 0;
    last_capture_time = HAL_GetTick();
    signal_detected = 0;
    high_freq_mode = 0;
    
    // Настраиваем TIM8 в режим Input Capture
    // TIM8_CH1 на PC6 - захват по переднему фронту
    
    // Отключаем все выходные таймеры
    HAL_TIM_Base_Stop_IT(&htim1);
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);
    
    // Настраиваем TIM8 как Master для Input Capture
    TIM8->CR1 &= ~TIM_CR1_CEN;  // Останавливаем
    TIM8->PSC = 0;              // Без делителя для максимальной точности
    TIM8->ARR = 0xFFFF;         // Максимальный период
    TIM8->CNT = 0;
    
    // Настраиваем канал 1 на Input Capture
    TIM8->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM8->CCMR1 |= TIM_CCMR1_CC1S_0;  // CC1 как вход, mapped на TI1
    
    // Захват по переднему фронту
    TIM8->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    
    // Включаем захват на канале 1
    TIM8->CCER |= TIM_CCER_CC1E;
    
    // Включаем прерывание по захвату
    TIM8->DIER |= TIM_DIER_CC1IE;
    
    // Настраиваем TIM8 как Trigger Output для других таймеров
    // OC1REF используется как TRGO
    TIM8->CR2 &= ~TIM_CR2_MMS;
    TIM8->CR2 |= TIM_CR2_MMS_2 | TIM_CR2_MMS_1;  // Compare Pulse (OC1REF)
    
    // Настраиваем режим Toggle для OC1
    TIM8->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM8->CCMR1 |= (0x3 << TIM_CCMR1_OC1M_Pos);  // Toggle mode
    TIM8->CCR1 = 0;  // Сразу же переключать
    
    // Запускаем TIM8
    TIM8->EGR |= TIM_EGR_UG;  // Update event
    TIM8->CR1 |= TIM_CR1_CEN;
    
    // Настраиваем TIM1, TIM3, TIM4 в Slave Mode
    // Они будут переключать выход по сигналу от TIM8
    configure_slave_timer(&htim1, TIM1);
    configure_slave_timer(&htim3, TIM3);
    configure_slave_timer(&htim4, TIM4);
    
    // Включаем прерывание TIM8
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);  // Наивысший приоритет
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    
    my_printf("[ANALOG] TIM8 Input Capture configured\n");
    my_printf("[ANALOG] Waiting for signal on PC6...\n");
}

// ============================================
// НАСТРОЙКА SLAVE ТАЙМЕРА
// ============================================

void configure_slave_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *TIMx)
{
    // Останавливаем таймер
    TIMx->CR1 &= ~TIM_CR1_CEN;
    
    // Настраиваем Slave Mode - External Clock Mode 1
    // Источник - ITR3 (для TIM1/3/4 это TIM8)
    TIMx->SMCR &= ~TIM_SMCR_TS;
    
    // TIM1: ITR2 = TIM8_TRGO
    // TIM3: ITR2 = TIM8_TRGO  
    // TIM4: ITR2 = TIM8_TRGO
    TIMx->SMCR |= (0x2 << TIM_SMCR_TS_Pos);  // ITR2
    
    // Slave Mode: External Clock Mode 1
    TIMx->SMCR &= ~TIM_SMCR_SMS;
    TIMx->SMCR |= (0x7 << TIM_SMCR_SMS_Pos);  // External Clock Mode 1
    
    // Без делителя
    TIMx->PSC = 0;
    TIMx->ARR = 1;  // Toggle на каждом фронте от TIM8
    
    // Output Compare Toggle mode
    if(TIMx == TIM1) {
        // TIM1_CH1 (PA8)
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
        TIMx->CCMR1 |= (0x3 << TIM_CCMR1_OC1M_Pos);  // Toggle
        TIMx->CCER |= TIM_CCER_CC1E;
    }
    else if(TIMx == TIM3) {
        // TIM3_CH1 (PA6)
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
        TIMx->CCMR1 |= (0x3 << TIM_CCMR1_OC1M_Pos);  // Toggle
        TIMx->CCER |= TIM_CCER_CC1E;
    }
    else if(TIMx == TIM4) {
        // TIM4_CH1 (PB6)
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
        TIMx->CCMR1 |= (0x3 << TIM_CCMR1_OC1M_Pos);  // Toggle
        TIMx->CCER |= TIM_CCER_CC1E;
    }
    
    TIMx->CCR1 = 0;
    
    // Запускаем таймер
    TIMx->CNT = 0;
    TIMx->EGR |= TIM_EGR_UG;
    TIMx->CR1 |= TIM_CR1_CEN;
}

// ============================================
// ОБРАБОТКА INPUT CAPTURE
// ============================================

void analog_follower_capture_callback(void)
{
    static uint32_t last_capture_value = 0;
    uint32_t current_capture = TIM8->CCR1;
    
    capture_count++;
    signal_detected = 1;
    
    // Вычисляем период
    uint32_t period;
    if(current_capture >= last_capture_value) {
        period = current_capture - last_capture_value;
    } else {
        period = (0xFFFF - last_capture_value) + current_capture;
    }
    
    last_capture_value = current_capture;
    
    // Вычисляем частоту (150 МГц / period)
    if(period > 0) {
        current_frequency = 150000000 / period;
        
        // Если частота >500 кГц - переходим в high-freq режим
        if(current_frequency > 500000 && !high_freq_mode) {
            enter_high_frequency_mode();
        }
        else if(current_frequency <= 500000 && high_freq_mode) {
            exit_high_frequency_mode();
        }
    }
}

// ============================================
// ПЕРЕХОД В РЕЖИМ ВЫСОКОЙ ЧАСТОТЫ
// ============================================

void enter_high_frequency_mode(void)
{
    high_freq_mode = 1;
    
    // ============================================
    // КРИТИЧНО: НЕ используем __disable_irq()!
    // ============================================
    // Отключаем только НИЗКОПРИОРИТЕТНЫЕ прерывания
    // FDCAN (prio 0) и TIM8 (prio 5) ОСТАЮТСЯ АКТИВНЫМИ
    
    // Отключаем UART (prio 10) - тормозит
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    
    // Отключаем таймеры вывода (prio 10)
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    
    // Отключаем TIM6 (системный, prio 6)
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    
    // FDCAN (prio 0) и TIM8_CC (prio 5) РАБОТАЮТ!
    
    my_printf("[ANALOG] HIGH FREQ MODE (>500kHz): low-priority IRQs disabled\n");
    my_printf("[ANALOG] FDCAN and TIM8 remain active\n");
}

// ============================================
// ВЫХОД ИЗ РЕЖИМА ВЫСОКОЙ ЧАСТОТЫ  
// ============================================

void exit_high_frequency_mode(void)
{
    high_freq_mode = 0;
    
    // Восстанавливаем все прерывания
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    
    my_printf("[ANALOG] Normal mode (<500kHz): all interrupts enabled\n");
}

// ============================================
// ОСТАНОВКА АНАЛОГОВОГО РЕЖИМА
// ============================================

void analog_follower_stop(void)
{
    my_printf("[ANALOG] Stopping analog follower\n");
    
    // Останавливаем TIM8
    TIM8->CR1 &= ~TIM_CR1_CEN;
    TIM8->DIER &= ~TIM_DIER_CC1IE;
    
    // Отключаем прерывание
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
    
    // Останавливаем slave таймеры
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // Восстанавливаем прерывания если были отключены
    if(high_freq_mode) {
        __enable_irq();
        high_freq_mode = 0;
    }
    
    signal_detected = 0;
    current_frequency = 0;
}

// ============================================
// ОБРАБОТКА В ГЛАВНОМ ЦИКЛЕ
// ============================================

void analog_follower_process(void)
{
    static uint32_t last_check = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Проверяем раз в секунду
    if(current_time - last_check < 1000) {
        return;
    }
    last_check = current_time;
    
    // Проверяем наличие сигнала
    if(capture_count == 0) {
        signal_detected = 0;
        g_system_state.analog_signal_present = 0;
    } else {
        g_system_state.analog_signal_present = 1;
        
        // Выводим статистику (не в high-freq режиме)
        if(!high_freq_mode) {
            my_printf("[ANALOG] Freq: %lu Hz, Captures: %lu\n", 
                      current_frequency, capture_count);
        }
    }
    
    capture_count = 0;  // Сбрасываем счётчик
}

// ============================================
// ПОЛУЧЕНИЕ ТЕКУЩЕЙ ЧАСТОТЫ
// ============================================

uint32_t analog_follower_get_frequency(void)
{
    return current_frequency;
}

uint8_t analog_follower_has_signal(void)
{
    return signal_detected;
}
