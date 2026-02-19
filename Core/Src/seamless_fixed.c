/**
 * seamless_fixed.c — Seamless frequency switching for FIXED_FREQUENCY mode
 * Version: 1.0
 *
 * Поддерживаемые таймеры:
 *   TIM2  — 32-бит, hardware OC toggle (PA15, FR channel)
 *   TIM1  — 16-бит, interrupt + manual GPIO toggle (PA8,  FL channel)
 *   TIM3  — 16-бит, interrupt + manual GPIO toggle (PA6,  RL channel)
 *   TIM4  — 16-бит, interrupt + manual GPIO toggle (PB9,  RR channel)
 */

#include "seamless_fixed.h"
#include "main.h"
#include <math.h>

#define APB1_CLK  150000000UL

extern void my_printf(const char *fmt, ...);

/* ============================================================
 *  ПУБЛИЧНЫЕ ФУНКЦИИ
 * ============================================================ */

void seamless_update_tim2(uint16_t new_psc, uint32_t new_arr)
{
    uint16_t old_psc = (uint16_t)TIM2->PSC;
    uint32_t old_arr = TIM2->ARR;
    uint32_t cnt     = TIM2->CNT;

    my_printf("[SEAMLESS TIM2] PSC:%u->%u  ARR:%lu->%lu  CNT=%lu\n",
              old_psc, new_psc, old_arr, new_arr, cnt);

    /*
     * PSC: просто пишем. Регистр всегда буферизован аппаратно.
     *      Новое значение попадёт в shadow при следующем UPDATE event.
     *
     * ARR: включаем ARPE и пишем. При ARPE=1 значение тоже идёт в preload.
     *      Оба регистра применятся ОДНОВРЕМЕННО при одном UPDATE event.
     *
     * Текущий период доживёт до old_arr с old_psc.
     * Следующий период начнётся с new_arr и new_psc.
     *
     * TIM2 — hardware OC toggle, CPU не участвует. Гонок нет.
     */
    TIM2->PSC       = new_psc;
    TIM2->CR1      |= TIM_CR1_ARPE;
    TIM2->ARR       = new_arr;

    my_printf("[SEAMLESS TIM2] Done. Will apply at next natural UPDATE.\n");
}

void seamless_update_tim16(TIM_TypeDef *tim, const char *name,
                            uint16_t new_psc, uint16_t new_arr)
{
    uint16_t old_psc = (uint16_t)tim->PSC;
    uint16_t old_arr = (uint16_t)tim->ARR;
    uint16_t cnt     = (uint16_t)tim->CNT;

    my_printf("[SEAMLESS %s] PSC:%u->%u  ARR:%u->%u  CNT=%u\n",
              name, old_psc, new_psc, old_arr, new_arr, cnt);

    /*
     * Та же логика что и для TIM2.
     *
     * Для TIM1/3/4 UPDATE event вызывает прерывание →
     * HAL_TIM_PeriodElapsedCallback → HAL_GPIO_TogglePin.
     * Этот toggle является ПРАВИЛЬНЫМ: это конец последнего периода
     * старой частоты. Лишних toggles нет.
     *
     * ВАЖНО: UG (EGR_UG) намеренно НЕ используется.
     *   UG = принудительный UPDATE = прерывание = внеплановый toggle = глитч.
     *   PSC буферизован аппаратно, ARR буферизован через ARPE.
     *   Оба применятся при естественном переполнении счётчика.
     */
    tim->PSC   = new_psc;
    tim->CR1  |= TIM_CR1_ARPE;
    tim->ARR   = new_arr;

    my_printf("[SEAMLESS %s] Done. Will apply at next natural UPDATE.\n", name);
}

/* ============================================================
 *  РАСЧЁТ PSC / ARR
 * ============================================================ */

void calc_psc_arr_32bit(float freq_hz, uint16_t *psc, uint32_t *arr)
{
    /*
     * Для ONLY_FR (TIM2, 32-бит): ищем минимальный PSC, дающий нулевую или
     * минимальную ошибку. PSC=0 (делитель 1) даёт максимальное разрешение:
     * ARR = APB1_CLK / freq_hz - 1 (до 2^32-1 = 4 294 967 295).
     *
     * При APB1=150MHz и output=1Hz:
     *   f_timer = 2 Hz → ARR = 150M/2 - 1 = 74 999 999 → OK для 32-бит.
     *
     * При output=2250Hz (максимум при PSC=0):
     *   f_timer = 4500 Hz → ARR = 150M/4500 - 1 = 33332 → OK.
     *
     * Для максимальной точности (3 знака после запятой для ONLY_FR)
     * перебираем PSC начиная с 0 и берём первый с минимальной ошибкой.
     * Поскольку при PSC=0 ARR = 74,999,999 max, ошибка обычно 0.
     */
    if (freq_hz <= 0.0f) {
        *psc = 0;
        *arr = 1;
        return;
    }

    double target_ticks = (double)APB1_CLK / (double)freq_hz;

    double   best_error = 1e18;
    uint16_t best_psc   = 0;
    uint32_t best_arr   = 1;

    for (uint32_t p = 0; p <= 65535; p++) {
        double arr_f = target_ticks / (double)(p + 1) - 1.0;

        if (arr_f < 1.0)              break;   /* дальше только хуже */
        if (arr_f > 4294967295.0)     continue;/* не влезает в 32-бит */

        uint32_t arr_u = (uint32_t)(arr_f + 0.5); /* округление */
        if (arr_u < 1) arr_u = 1;

        double actual  = (double)(p + 1) * (double)(arr_u + 1);
        double err     = fabs(actual - target_ticks);

        if (err < best_error) {
            best_error = err;
            best_psc   = (uint16_t)p;
            best_arr   = arr_u;

            if (err == 0.0) break;  /* точное решение найдено */
        }
    }

    *psc = best_psc;
    *arr = best_arr;

    /* Диагностика */
    double actual_f_timer = (double)APB1_CLK /
                            ((double)(best_psc + 1) * (double)(best_arr + 1));
    my_printf("[CALC 32b] target_timer=%.3f Hz  PSC=%u  ARR=%lu"
              "  actual_timer=%.3f Hz  out=%.3f Hz\n",
              freq_hz, best_psc, best_arr,
              actual_f_timer, actual_f_timer / 2.0);
}

void calc_psc_arr_16bit(float freq_hz, uint16_t *psc, uint16_t *arr)
{
    /*
     * Для ALL_FOUR (TIM1/3/4, 16-бит): ARR ограничен 65535.
     * Ищем комбинацию с минимальной ошибкой частоты.
     * Начинаем с PSC=0 — если ARR влезает в 16 бит, используем его
     * (максимальное разрешение).
     *
     * При APB1=150MHz и output=1Hz:
     *   f_timer=2Hz → target_ticks=75M → ARR=74999999 > 65535.
     *   Нужен PSC: 75M / (PSC+1) - 1 ≤ 65535 → PSC+1 ≥ 75M/65536 ≈ 1144 → PSC≥1143.
     *   PSC=1144 (1145): ARR=75M/1145-1=65502 (ОК).
     */
    if (freq_hz <= 0.0f) {
        *psc = 0;
        *arr = 1;
        return;
    }

    double target_ticks = (double)APB1_CLK / (double)freq_hz;

    double   best_error = 1e18;
    uint16_t best_psc   = 0;
    uint16_t best_arr   = 1;

    for (uint32_t p = 0; p <= 65535; p++) {
        double arr_f = target_ticks / (double)(p + 1) - 1.0;

        if (arr_f < 1.0)    break;   /* ARR будет только меньше с большим PSC */
        if (arr_f > 65535.0) continue;

        uint16_t arr_u = (uint16_t)(arr_f + 0.5);
        if (arr_u < 1) arr_u = 1;

        double actual = (double)(p + 1) * (double)(arr_u + 1);
        double err    = fabs(actual - target_ticks);

        if (err < best_error) {
            best_error = err;
            best_psc   = (uint16_t)p;
            best_arr   = arr_u;

            if (err == 0.0) break;
        }
    }

    *psc = best_psc;
    *arr = best_arr;

    double actual_f_timer = (double)APB1_CLK /
                            ((double)(best_psc + 1) * (double)(best_arr + 1));
    my_printf("[CALC 16b] target_timer=%.3f Hz  PSC=%u  ARR=%u"
              "  actual_timer=%.3f Hz  out=%.3f Hz\n",
              freq_hz, best_psc, best_arr,
              actual_f_timer, actual_f_timer / 2.0);
}
