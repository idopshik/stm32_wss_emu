#include "wheel_control.h"
#include "system_modes.h"  // ✅ ДОБАВИТЬ - там есть g_system_state
#include <stdlib.h>
#include <stdio.h>

extern void my_printf(const char *fmt, ...);
// Внешние таймеры (объявлены в main.c)
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// Статические переменные (видимы только в этом файле)
static whl_chnl fl_wheel = {0, 24, 0, 0};
static whl_chnl fr_wheel = {0, 12, 0, 0};
static whl_chnl rl_wheel = {0, 24, 0, 0};
static whl_chnl rr_wheel = {0, 24, 0, 0};

// Глобальный массив указателей (extern объявлен в .h)
whl_chnl *whl_arr[4] = {NULL};

// Константы для индексов (только в этом файле)
enum {
    numFL = 0,
    numFR = 1,
    numRL = 2,
    numRR = 3
};

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================

/*
 * ФИЗИЧЕСКАЯ ЛОГИКА:
 *   ABS датчик: 48 зубьев × 2 импульса/зуб = 96 импульсов/оборот
 *   f_signal_Hz  = raw_RPM × 0.032          (выход GPIO, TOGGLE ÷2 уже учтён)
 *   f_timer_Hz   = raw_RPM × 0.064          (частота таймера = f_signal × 2)
 *   Total_Divider = APB1_CLK / f_timer_Hz
 *                 = 150 000 000 / (raw × 0.064)
 *                 = 2 343 750 000 / raw
 *
 * ОШИБКА СТАРОГО КОДА: использовал val * 0.02 * MinutTeethFactor = val * 0.032
 *   => делал APB1_CLK / (val * 0.032 * (PSC+1)) = 4 687 500 000 / (val*(PSC+1))
 *   => ARR в 2 раза больше => выходная частота в 2 раза ниже требуемой
 *
 * ИСПРАВЛЕНИЕ: явная константа 2 343 750 000
 */

#define TOTAL_DIVIDER_CONST  2343750000ULL   // APB1_CLK / (raw * 0.064)

uint32_t calculete_period_only(int val)
{
    // For 32-bit TIM2 (PSC = 12 fixed)
    // ARR = Total_Divider / (PSC + 1) - 1 = 2343750000 / (val * 13) - 1
    uint64_t total_divider = TOTAL_DIVIDER_CONST / (uint32_t)val;
    return (uint32_t)(total_divider / (12 + 1)) - 1;
}

int calculete_prsc_and_perio(int val, int *arr, int wheelnum)
{
    int newpresc;

    if (val < 4274) {
        newpresc = 1200;
    }
    else {
        newpresc = 24;
    }

    // Только временно
    if (whl_arr[wheelnum]->initial_tmp_flag == 0) {
        whl_arr[wheelnum]->initial_tmp_flag = 1;
        arr[0] = 24;  // tmp
    }
    else {
        arr[0] = whl_arr[wheelnum]->prev_psc;
    }
    whl_arr[wheelnum]->prev_psc = newpresc;

    arr[1] = newpresc;

    // FIX: используем корректную константу Total_Divider = 2 343 750 000 / val
    // Старый код: APB1_CLK / (val * 0.02 * MinutTeethFactor * (PSC+1)) давал
    //   Total_Divider / (PSC+1) с константой 4 687 500 000 — в 2 раза больше нужной.
    uint64_t total_divider = TOTAL_DIVIDER_CONST / (uint32_t)val;

    uint32_t tmp;
    tmp = (uint32_t)(total_divider / (uint32_t)(arr[0] + 1)) - 1;

    if (tmp > 65535) {
        tmp = 65535;  // обрезаем. Хоть это и не правильно
    }

    arr[2] = (int)tmp;

    if (arr[0] != arr[1]) {
        tmp = (uint32_t)(total_divider / (uint32_t)(newpresc + 1)) - 1;

        if (tmp > 65535) {
            tmp = 65535;  // обрезаем. Хоть это и не правильно.
        }

        arr[3] = (int)tmp;
    }
    else {
        arr[3] = arr[2];
    }

    return 0;
}

// ==================== ОСНОВНЫЕ ФУНКЦИИ ====================

void wheel_control_init(void)
{
    // Инициализация структур
    fl_wheel.prev_speed = 0;
    fl_wheel.prev_psc = 24;
    fl_wheel.initial_tmp_flag = 0;
    fl_wheel.psc_change_flag = 0;
    
    fr_wheel.prev_speed = 0;
    fr_wheel.prev_psc = 12;
    fr_wheel.initial_tmp_flag = 0;
    fr_wheel.psc_change_flag = 0;
    
    rl_wheel.prev_speed = 0;
    rl_wheel.prev_psc = 24;
    rl_wheel.initial_tmp_flag = 0;
    rl_wheel.psc_change_flag = 0;
    
    rr_wheel.prev_speed = 0;
    rr_wheel.prev_psc = 24;
    rr_wheel.initial_tmp_flag = 0;
    rr_wheel.psc_change_flag = 0;
    
    // Настройка массива указателей
    whl_arr[numFL] = &fl_wheel;
    whl_arr[numFR] = &fr_wheel;
    whl_arr[numRL] = &rl_wheel;
    whl_arr[numRR] = &rr_wheel;
    
    my_printf("Wheel control initialized\n");
}

void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm)
{
    /* my_printf("settins speeed ...\n"); */


    // Проверка инициализации
    if (whl_arr[0] == NULL || whl_arr[1] == NULL || 
        whl_arr[2] == NULL || whl_arr[3] == NULL) {
        printf("ERROR: Wheel array not initialized! Calling init...\n");
        wheel_control_init();
    }
    
    // ✅ ОБНОВЛЯЕМ МАСКУ ПО ФАКТУ АКТИВНОСТИ КАНАЛОВ
    g_system_state.channel_mask = 0;
    if (vFLrpm >= 0x3F) g_system_state.channel_mask |= 0x01;
    if (vFRrpm >= 0x3F) g_system_state.channel_mask |= 0x02;
    if (vRLrpm >= 0x3F) g_system_state.channel_mask |= 0x04;
    if (vRRrpm >= 0x3F) g_system_state.channel_mask |= 0x08;
    int arr_with_calculations[4] = {300, 300, 300, 300};

    ////////////    TIMER1 -  FL  ///////////////

    if (vFLrpm < 0x3F) {
        TIM1->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM1->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vFLrpm, arr_with_calculations, numFL);

        if (vFLrpm != whl_arr[numFL]->prev_speed) {
            if (vFLrpm > whl_arr[numFL]->prev_speed) {
                if (whl_arr[numFL]->psc_change_flag == 1) {
                    // this isn't functional now.
                    whl_arr[numFL]->psc_change_flag = 0;
                    TIM1->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM1->CNT > (arr_with_calculations[2])) {
                    TIM1->PSC = arr_with_calculations[1];
                    TIM1->ARR = arr_with_calculations[3];
                    TIM1->EGR = TIM_EGR_UG;
                }
                else {
                    TIM1->CR1 |= TIM_CR1_ARPE;
                    TIM1->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM1->CR1 &= ~TIM_CR1_ARPE;
                        TIM1->PSC = arr_with_calculations[1];
                        TIM1->ARR = arr_with_calculations[3];
                        TIM1->EGR = TIM_EGR_UG;
                    }
                }
            }
            else {
                TIM1->CR1 |= TIM_CR1_ARPE;
                TIM1->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    TIM1->PSC = arr_with_calculations[1];
                    TIM1->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numFL]->prev_speed = vFLrpm;
        }
    }

    ////////////    TIMER2 -  FR  ///////////////
    if (vFRrpm < 0x3F) {
        TIM2->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM2->CR1 |= TIM_CR1_CEN;  // enable
        
        uint32_t current_32bit_period = calculete_period_only(vFRrpm);

        if (vFRrpm > whl_arr[numFR]->prev_speed) {
            TIM2->CR1 |= TIM_CR1_ARPE;
            TIM2->ARR = current_32bit_period;

            if (TIM2->CNT > current_32bit_period) {
                TIM2->EGR |= TIM_EGR_UG;
            }
        }
        else {
            if (TIM2->CNT < current_32bit_period) {
                TIM2->CR1 |= TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
            else {
                TIM2->CR1 &= ~TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
        }

        // Костыльный предохранитель от убегания.
        if (TIM2->CNT > current_32bit_period) {
            TIM2->EGR |= TIM_EGR_UG;
        }
    }
    whl_arr[numFR]->prev_speed = vFRrpm;

    ////////////    TIMER3 -  RL  ///////////////

    if (vRLrpm < 0x3F) {
        TIM3->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM3->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vRLrpm, arr_with_calculations, numRL);

        if (vRLrpm != whl_arr[numRL]->prev_speed) {
            if (vRLrpm > whl_arr[numRL]->prev_speed) {
                if (whl_arr[numRL]->psc_change_flag == 1) {
                    whl_arr[numRL]->psc_change_flag = 0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM3->CNT > (arr_with_calculations[2])) {
                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];
                    TIM3->EGR = TIM_EGR_UG;
                }
                else {
                    TIM3->CR1 |= TIM_CR1_ARPE;
                    TIM3->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM3->CR1 &= ~TIM_CR1_ARPE;
                        TIM3->PSC = arr_with_calculations[1];
                        TIM3->ARR = arr_with_calculations[3];
                        TIM3->EGR = TIM_EGR_UG;
                    }
                }
            }
            else {
                TIM3->CR1 |= TIM_CR1_ARPE;
                TIM3->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numRL]->prev_speed = vRLrpm;
        }
    }

    ////////////    TIMER4 -  RR  ///////////////

    if (vRRrpm < 0x3F) {
        TIM4->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM4->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vRRrpm, arr_with_calculations, numRR);

        if (vRRrpm != whl_arr[numRR]->prev_speed) {
            if (vRRrpm > whl_arr[numRR]->prev_speed) {
                if (whl_arr[numRR]->psc_change_flag == 1) {
                    whl_arr[numRR]->psc_change_flag = 0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM4->CNT > (arr_with_calculations[2])) {
                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];
                    TIM4->EGR = TIM_EGR_UG;
                }
                else {
                    TIM4->CR1 |= TIM_CR1_ARPE;
                    TIM4->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM4->CR1 &= ~TIM_CR1_ARPE;
                        TIM4->PSC = arr_with_calculations[1];
                        TIM4->ARR = arr_with_calculations[3];
                        TIM4->EGR = TIM_EGR_UG;
                    }
                }
            }
            else {
                TIM4->CR1 |= TIM_CR1_ARPE;
                TIM4->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numRR]->prev_speed = vRRrpm;
        }
    }
}
