/**
 * analog_follower.h
 * 
 * Аналоговый режим следования внешнему сигналу
 */

#ifndef ANALOG_FOLLOWER_H
#define ANALOG_FOLLOWER_H

#include "main.h"
#include <stdint.h>

// ============================================
// ФУНКЦИИ УПРАВЛЕНИЯ
// ============================================

void analog_follower_init(void);
void analog_follower_start(void);
void analog_follower_stop(void);
void analog_follower_process(void);

// ============================================
// ОБРАБОТЧИКИ
// ============================================

void analog_follower_capture_callback(void);
void configure_slave_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *TIMx);
void enter_high_frequency_mode(void);
void exit_high_frequency_mode(void);

// ============================================
// ПОЛУЧЕНИЕ ИНФОРМАЦИИ
// ============================================

uint32_t analog_follower_get_frequency(void);
uint8_t analog_follower_has_signal(void);

#endif // ANALOG_FOLLOWER_H
