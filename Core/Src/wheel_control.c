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

uint32_t calculete_period_only(int val)
{
    // For 32bit timer
    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;  // значение регистра ARR
}

int calculete_prsc_and_perio(int val, int *arr, int wheelnum)
{
    float factor = 0.02;
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

    uint32_t tmp;
    tmp = (APB1_CLK / (val * factor * MinutTeethFactor * (arr[0] + 1))) - 1;  // значение регистра ARR
    
    if (tmp > 65536) {
        tmp = 65535;  // обрезаем. Хоть это и не правильно
    }
    
    arr[2] = (int)tmp;
    
    if (arr[0] != arr[1]) {
        tmp = ((APB1_CLK / (val * factor * MinutTeethFactor * (newpresc + 1))) - 1);  // значение регистра ARR

        if (tmp > 65536) {
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
