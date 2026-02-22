/**
 * @file wss_dds.c
 * @brief DMA-based WSS signal generator for STM32G431
 *
 * ═══════════════════════════════════════════════════════════════
 *  ARCHITECTURE
 * ═══════════════════════════════════════════════════════════════
 *
 *  Hardware (zero CPU, zero jitter):
 *    TIM2 update @ 1 MHz → DMA1_Ch2 → GPIOA->BSRR
 *
 *  Software (main loop, non-critical):
 *    DMA ISR → sets flag (< 20 cycles, never blocks)
 *    main loop → wss_dds_process() → fill_buffer()
 *    Pre-computes 1024 BSRR values per half
 *
 *  Buffer: 2048 entries = 2.048 ms circular
 *  Each half: 1024 µs to fill → ~340 µs @ -O2 → 67% margin
 *
 * ═══════════════════════════════════════════════════════════════
 *  FORMULA (1 µs sample period, 1 MHz DMA rate)
 * ═══════════════════════════════════════════════════════════════
 *
 *  DDS phase: one revolution = 2^32 phase units
 *  48 toggle points per revolution (48 teeth, 1 edge per tooth for meander)
 *  phase_per_toggle = 2^32 / 48 = 89,478,485
 *
 *  phase_step per sample = (RPM / 60) × 2^32 / sample_rate
 *    RPM = raw × 0.02              (DBC factor)
 *    step = raw × 0.02 / 60 × 2^32 / 1,000,000
 *         = raw × 2^32 / 3,000,000,000
 *
 *  Verification (raw = 42850 → 100 km/h, wheel Ø1945 mm):
 *    RPM = 42850 × 0.02 = 857 RPM
 *    step = 42850 × 4294967296 / 3000000000 = 61,344
 *    Samples/rev = 2^32 / 61344 = 70,000
 *    Time/rev = 70,000 µs = 0.070 s
 *    RPM = 60 / 0.070 = 857.1 ✓
 *    f_toggle = 857 × 48 / 60 = 685.6 Hz
 *    tooth_period = 1 / 685.6 = 1.459 ms  ✓
 */

#include "wss_dds.h"

/* ── Buffer ────────────────────────────────────────────────────── */

#define DMA_BUF_LEN     2048
#define DMA_HALF_LEN    (DMA_BUF_LEN / 2)  /* 1024 */

static uint32_t wss_dma_buf[DMA_BUF_LEN] __attribute__((aligned(4)));

/* ── Flags: ISR sets, main loop clears ─────────────────────────── */

static volatile uint8_t need_fill_first;
static volatile uint8_t need_fill_second;

/* ── DDS constants ─────────────────────────────────────────────── */

#define WSS_TEETH_PER_REV   48
#define TOGGLES_PER_REV     WSS_TEETH_PER_REV     /* 48 teeth = 48 meander periods */
#define PHASE_MAX           (1ULL << 32)
#define PHASE_PER_TOGGLE    ((uint32_t)(PHASE_MAX / TOGGLES_PER_REV))

/*
 * step = raw × 2^32 / STEP_DIVISOR
 *
 * STEP_DIVISOR = 60 × sample_rate / DBC_factor
 *              = 60 × 1,000,000 / 0.02
 *              = 3,000,000,000
 *
 * This encodes: DBC factor (0.02 RPM), RPM→rev/s (/60), sample rate (1 MHz)
 */
#define STEP_DIVISOR        3000000000ULL

/* ── DDS channel state ─────────────────────────────────────────── */

typedef struct {
    uint32_t phase;
    uint32_t step;          /* updated from CAN ISR, read by fill */
    uint32_t next_edge;
    uint8_t  level;
    uint16_t pin;
} wss_ch_t;

static wss_ch_t ch[WSS_NUM_CHANNELS];

/* ── fill_buffer ───────────────────────────────────────────────── *
 *
 * Pragma forces -O2 even when project is -O0/-Og.
 * Channels are manually unrolled with local variables to minimize
 * memory traffic (critical if pragma is somehow ignored).
 *
 * Budget: 1024 samples @ ~50 cyc/sample = 51,200 cycles
 * Available: 1024 µs × 170 MHz = 174,080 cycles → 71% margin
 * ──────────────────────────────────────────────────────────────── */

#pragma GCC push_options
#pragma GCC optimize ("O2")

static void fill_buffer(uint32_t *buf, uint32_t len)
{
    /* Load ALL state into locals — avoids struct memory access overhead */
    uint32_t ph0 = ch[0].phase, st0 = ch[0].step, ne0 = ch[0].next_edge;
    uint32_t lv0 = ch[0].level, pn0 = ch[0].pin;

    uint32_t ph1 = ch[1].phase, st1 = ch[1].step, ne1 = ch[1].next_edge;
    uint32_t lv1 = ch[1].level, pn1 = ch[1].pin;

    uint32_t ph2 = ch[2].phase, st2 = ch[2].step, ne2 = ch[2].next_edge;
    uint32_t lv2 = ch[2].level, pn2 = ch[2].pin;

    uint32_t ph3 = ch[3].phase, st3 = ch[3].step, ne3 = ch[3].next_edge;
    uint32_t lv3 = ch[3].level, pn3 = ch[3].pin;

    for (uint32_t i = 0; i < len; i++) {

        uint32_t bsrr = 0;

        /* ── Channel 0 (PA8 FL) ────────────────────────────── */
        if (st0 == 0) {
            bsrr = pn0 << 16;                  /* LOW */
        } else {
            ph0 += st0;
            if ((int32_t)(ph0 - ne0) >= 0) {
                ne0 += PHASE_PER_TOGGLE;
                lv0 ^= 1;
            }
            bsrr = lv0 ? pn0 : (pn0 << 16);
        }

        /* ── Channel 1 (PA15 FR) ────────────────────────────── */
        if (st1 == 0) {
            bsrr |= pn1 << 16;
        } else {
            ph1 += st1;
            if ((int32_t)(ph1 - ne1) >= 0) {
                ne1 += PHASE_PER_TOGGLE;
                lv1 ^= 1;
            }
            bsrr |= lv1 ? pn1 : (pn1 << 16);
        }

        /* ── Channel 2 (PA0 RL) ────────────────────────────── */
        if (st2 == 0) {
            bsrr |= pn2 << 16;
        } else {
            ph2 += st2;
            if ((int32_t)(ph2 - ne2) >= 0) {
                ne2 += PHASE_PER_TOGGLE;
                lv2 ^= 1;
            }
            bsrr |= lv2 ? pn2 : (pn2 << 16);
        }

        /* ── Channel 3 (PA6 RR) ────────────────────────────── */
        if (st3 == 0) {
            bsrr |= pn3 << 16;
        } else {
            ph3 += st3;
            if ((int32_t)(ph3 - ne3) >= 0) {
                ne3 += PHASE_PER_TOGGLE;
                lv3 ^= 1;
            }
            bsrr |= lv3 ? pn3 : (pn3 << 16);
        }

        buf[i] = bsrr;
    }

    /* Write back state (step is read-only here, updated by CAN) */
    ch[0].phase = ph0; ch[0].next_edge = ne0; ch[0].level = lv0;
    ch[1].phase = ph1; ch[1].next_edge = ne1; ch[1].level = lv1;
    ch[2].phase = ph2; ch[2].next_edge = ne2; ch[2].level = lv2;
    ch[3].phase = ph3; ch[3].next_edge = ne3; ch[3].level = lv3;
}

#pragma GCC pop_options

/* ── Raw CAN → phase step ──────────────────────────────────────── */

static uint32_t raw_to_step(uint16_t raw)
{
    if (raw == 0) return 0;
    return (uint32_t)( ((uint64_t)raw * PHASE_MAX) / STEP_DIVISOR );
}

/* ── Public API ────────────────────────────────────────────────── */

void wss_dds_init(void)
{
    ch[WSS_FL].pin = GPIO_PIN_8;   // PA8
    ch[WSS_FR].pin = GPIO_PIN_15;  // PA15
    ch[WSS_RL].pin = GPIO_PIN_0;   // PA0
    ch[WSS_RR].pin = GPIO_PIN_6;   // PA6

    for (int i = 0; i < WSS_NUM_CHANNELS; i++) {
        ch[i].phase     = 0;
        ch[i].step      = 0;
        ch[i].next_edge = PHASE_PER_TOGGLE;
        ch[i].level     = 0;
        GPIOA->BSRR = (uint32_t)ch[i].pin << 16;  // Set LOW initially
    }

    fill_buffer(wss_dma_buf, DMA_BUF_LEN);
    need_fill_first  = 0;
    need_fill_second = 0;

    /* ── TIM2: 1 MHz ──────────────────────────────────────────── */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    __DSB();

    TIM2->CR1  = TIM_CR1_ARPE;
    TIM2->PSC  = 0;
    /* ARR для 1 MHz при SYSCLK=170 MHz:
     * f_update = 170 MHz / (PSC+1) / (ARR+1) = 170M / 1 / 170 = 1 MHz
     */
    TIM2->ARR  = (HAL_RCC_GetSysClockFreq() / 1000000U) - 1;  // 169 for 170 MHz
    TIM2->DIER = TIM_DIER_UDE;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;

    /* ── DMA1 Channel 2 → GPIOA->BSRR ────────────────────────── */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    __DSB();

    DMAMUX1_Channel1->CCR = DMA_REQUEST_TIM2_UP;

    DMA1_Channel2->CCR   = 0;
    DMA1_Channel2->CPAR  = (uint32_t)&GPIOA->BSRR;  // ← GPIOA instead of GPIOB
    DMA1_Channel2->CMAR  = (uint32_t)wss_dma_buf;
    DMA1_Channel2->CNDTR = DMA_BUF_LEN;

    DMA1_Channel2->CCR =
        DMA_CCR_CIRC    |
        DMA_CCR_DIR     |
        DMA_CCR_MINC    |
        DMA_CCR_MSIZE_1 |
        DMA_CCR_PSIZE_1 |
        DMA_CCR_PL_1 | DMA_CCR_PL_0 |
        DMA_CCR_HTIE    |
        DMA_CCR_TCIE    |
        DMA_CCR_EN;

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void wss_dds_set_rpm(const uint16_t raw[WSS_NUM_CHANNELS])
{
    for (int i = 0; i < WSS_NUM_CHANNELS; i++)
        ch[i].step = raw_to_step(raw[i]);
}

/* ── DMA ISR: flag only, < 20 cycles, never blocks ────────────── */

void wss_dds_dma_isr(void)
{
    uint32_t isr = DMA1->ISR;

    if (isr & DMA_ISR_HTIF2) {
        DMA1->IFCR = DMA_IFCR_CHTIF2;
        need_fill_first = 1;
    }

    if (isr & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        need_fill_second = 1;
    }
}

/* ── Main loop: call as often as possible ──────────────────────── */

void wss_dds_process(void)
{
    if (need_fill_first) {
        need_fill_first = 0;
        fill_buffer(&wss_dma_buf[0], DMA_HALF_LEN);
    }

    if (need_fill_second) {
        need_fill_second = 0;
        fill_buffer(&wss_dma_buf[DMA_HALF_LEN], DMA_HALF_LEN);
    }
}
