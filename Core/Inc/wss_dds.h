/**
 * @file wss_dds.h
 * @brief DMA-based WSS signal generator for STM32G431
 *
 * Architecture:
 *   TIM2 (1 MHz) → DMA1_Ch2 → GPIOA->BSRR (zero jitter)
 *   DMA ISR sets flag only (< 20 cycles)
 *   Main loop calls wss_dds_process() to refill buffer
 *
 * Outputs: PA8 (FL), PA15 (FR), PA0 (RL), PA6 (RR)
 *          All configured as GPIO_MODE_OUTPUT_OD for 5V external pull-up
 *
 * System clock: 170 MHz
 * APB1 clock:   170 MHz
 * Sample rate:  1 MHz (TIM2 ARR=169)
 */

#ifndef WSS_DDS_H
#define WSS_DDS_H

#include "main.h"
#include <stdint.h>

#define WSS_NUM_CHANNELS    4

#define WSS_FL  0
#define WSS_FR  1
#define WSS_RL  2
#define WSS_RR  3

/** Init TIM2 + DMA. Call after MX_GPIO_Init(). Starts DMA immediately. */
void wss_dds_init(void);

/** Set speeds from CAN. Safe to call from ISR. */
void wss_dds_set_rpm(const uint16_t raw[WSS_NUM_CHANNELS]);

/** DMA ISR — just clears flags + sets refill request. Call from DMA1_Channel2_IRQHandler. */
void wss_dds_dma_isr(void);

/** Call from main loop — does the actual buffer computation. Non-blocking if no work. */
void wss_dds_process(void);

#endif /* WSS_DDS_H */
