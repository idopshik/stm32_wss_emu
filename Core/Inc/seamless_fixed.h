#ifndef SEAMLESS_FIXED_H
#define SEAMLESS_FIXED_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/*
 * seamless_fixed.h — Seamless frequency switching for FIXED_FREQUENCY mode
 * Version: 1.0
 *
 * ПРИНЦИП (почему это работает без спинлоков и UG):
 *
 * На STM32 оба регистра буферизованы аппаратно:
 *
 *   PSC: ВСЕГДА буферизован (shadow register всегда активен).
 *        Запись → preload. В shadow попадает только при UPDATE event.
 *
 *   ARR: Буферизован только при ARPE=1 (TIM_CR1_ARPE).
 *        При ARPE=0 запись идёт напрямую в shadow (потенциальный глитч).
 *        При ARPE=1 запись → preload. В shadow при UPDATE event.
 *
 * Алгоритм:
 *   1. Записываем новый PSC (всегда идёт в preload)
 *   2. Ставим ARPE=1, записываем новый ARR (идёт в preload)
 *   3. Текущий период доживает до ARR_shadow_old → естественный UPDATE
 *   4. При UPDATE: оба значения одновременно грузятся из preload в shadow
 *
 * Для TIM2 (hardware OC toggle PA15):
 *   UPDATE event → аппаратура продолжает переключать пин с новой частотой.
 *   Никакого участия CPU, никаких гонок.
 *
 * Для TIM1/TIM3/TIM4 (interrupt + manual GPIO toggle):
 *   UPDATE event → прерывание → HAL_TIM_PeriodElapsedCallback → TogglePin.
 *   Это ПРАВИЛЬНЫЙ toggle (конец последнего периода старой частоты).
 *   Следующее прерывание — уже с новым периодом.
 *   Никакого лишнего UG, никаких лишних прерываний.
 *
 * Результат: один последний период на старой частоте, потом чистая новая частота.
 */

/**
 * @brief Seamless update для TIM2 (32-бит, hardware OC toggle на PA15)
 *
 * Оба параметра применятся одновременно на следующем естественном UPDATE event.
 * Никакого UG, никакого CNT сброса, никаких спинлоков.
 *
 * @param new_psc  Новый prescaler (0-based, real divisor = new_psc + 1)
 * @param new_arr  Новый auto-reload (32-бит)
 */
void seamless_update_tim2(uint16_t new_psc, uint32_t new_arr);

/**
 * @brief Seamless update для 16-битных таймеров (TIM1, TIM3, TIM4)
 *
 * Те же гарантии что и для TIM2.
 * Прерывание на UPDATE = один правильный toggle пина.
 *
 * @param tim      Указатель на таймер (TIM1, TIM3 или TIM4)
 * @param name     Имя для логов (например "TIM1")
 * @param new_psc  Новый prescaler
 * @param new_arr  Новый auto-reload (16-бит)
 */
void seamless_update_tim16(TIM_TypeDef *tim, const char *name,
                            uint16_t new_psc, uint16_t new_arr);

/**
 * @brief Расчёт оптимальных PSC/ARR для 32-битного таймера (TIM2, ONLY_FR)
 *
 * Перебирает все PSC от 0 до 65535, минимизирует ошибку частоты.
 * PSC=0 (делитель=1) даёт максимальное разрешение для 32-бит таймера.
 * Для 1000.000 Hz при APB1=150MHz: PSC=0, ARR=149999, ошибка=0.
 *
 * @param freq_hz  Частота ТАЙМЕРА в Гц (= output_Hz * 2)
 * @param psc      [out] Оптимальный PSC
 * @param arr      [out] Оптимальный ARR (32-бит)
 */
void calc_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr);

/**
 * @brief Расчёт оптимальных PSC/ARR для 16-битных таймеров (TIM1/3/4, ALL_FOUR)
 *
 * @param freq_hz  Частота ТАЙМЕРА в Гц (= output_Hz * 2)
 * @param psc      [out] Оптимальный PSC
 * @param arr      [out] Оптимальный ARR (16-бит, max 65535)
 */
void calc_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr);

#endif /* SEAMLESS_FIXED_H */
