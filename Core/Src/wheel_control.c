// wheel_control.c
#include "wheel_control.h"
#include "main.h"

// Объявить extern таймеры
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;



void wheel_control_init(void){
    // to do
}


void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[])
{
    //__HAL_TIM_SET_PRESCALER(&htim3, val );
    //__HAL_TIM_SET_AUTORELOAD(&htim3, val );
    /* permutations! */
    /* 1) changing prescaler up or down  */
    /* 2) the same prescaler */
    /* ----- */
    /* 1) going up (speed up) */
    /* 2) going down. */

    // 32-bit timers first
    //

    my_printf("enering new speed settings\n");
    my_printf("but not this time! return! \n");
    return 0;
    int arr_with_calculations[4] = {300, 300, 300, 300};

    ////////////    TIMER1 -  FL  ///////////////

    if (vFLrpm < 0x3F) {
        TIM1->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM1->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vFLrpm, arr_with_calculations, whl_arr, numFL);

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
                /*
                В любом случае рассчитываем точку по старому прескалеру.
                Если точка пройдена (по старому прескалеру) то.

                     - Если есть новое значение пресклера, кладём его в
                     PSC . Мы всё равно ведь будем вызывать  UG ?? оно как
                     раз применится.
                     - Кладём ARR (рассчитанный по новому)
                     - Вызываем UG.
                 */
                else {
                    /*
                     Точка ещё не пройдена. В этом случае нельзя вызвать UG.
                     будет Glitch.
                     - пишем новый ARR  по старому прескалеру (без буфера)
                     - если PSC поменялся, то кладём уже по буферу и PSC и ARR.
                     */
                    TIM1->CR1 |= TIM_CR1_ARPE;
                    TIM1->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM1->CR1 &= ~TIM_CR1_ARPE;
                        TIM1->PSC = arr_with_calculations[1];
                        TIM1->ARR = arr_with_calculations[3];

                        // только для теста. TODO
                        TIM1->EGR = TIM_EGR_UG;
                    }

                    /* TIM1->CR1 |= TIM_CR1_ARPE;    // будем писать сразу в ARR (мимо shadow) */
                    /* TIM1->ARR = arr_with_calculations[2]; */
                    /* [> Костыльный предохранитель от убегания. <] */
                    /* if (TIM1->CNT > arr_with_calculations[2]) { */
                    /* TIM1->EGR |= TIM_EGR_UG; */
                    /* } */
                }
            }
            else {
                /*
                меньше. Скорость уменьшается. В этом случае обычно.период
                увеличивается. Но - прескалер может
                увеличиться (переключиться), и тем самым рассчётное значение ARR
                может быть значительно меньше, чем текущее.
                1.  Если не меняется  PSC - пишем  ARR без буфер. Это не
                вызывает никакх проблем вообще
                2.  �?зменение  PSC в этом случае пишем по старому прескелеру
                без буфера, рассчитываем новую пару и пишем по буферу!
                 */

                TIM1->CR1 |= TIM_CR1_ARPE;
                TIM1->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    /* TIM1->CR1 &= ~TIM_CR1_ARPE; */

                    TIM1->PSC = arr_with_calculations[1];
                    TIM1->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numFL]->initial_tmp_flag = arr_with_calculations[1];
            whl_arr[numFL]->prev_speed = vFLrpm;
        }
    }

    ////////////    TIMER2 -  FR  ///////////////
    if (vFRrpm < 0x3F) {
        TIM2->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM2->CR1 |= TIM_CR1_CEN;  // enable
                                   //
        uint32_t current_32bit_period = calculete_period_only(vFRrpm);

        if (vFRrpm > whl_arr[numFR]->prev_speed) {
            TIM2->CR1 |= TIM_CR1_ARPE;
            TIM2->ARR = current_32bit_period;

            if (TIM2->CNT > current_32bit_period) {
                TIM2->EGR |= TIM_EGR_UG;
            }
        }
        else {
            /* my_printf(" ARR: %d\n\r", arr_with_calculations[0]); */

            if (TIM2->CNT < current_32bit_period) {
                TIM2->CR1 |= TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
            else {
                // Не должна никогда выполняться.
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

        calculete_prsc_and_perio(vRLrpm, arr_with_calculations, whl_arr, numRL);

        if (vRLrpm != whl_arr[numRL]->prev_speed) {
            if (vRLrpm > whl_arr[numRL]->prev_speed) {
                if (whl_arr[numRL]->psc_change_flag == 1) {
                    // this isn't functional now.
                    whl_arr[numRL]->psc_change_flag = 0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM3->CNT > (arr_with_calculations[2])) {
                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];
                    TIM3->EGR = TIM_EGR_UG;
                }
                /*
                В любом случае рассчитываем точку по старому прескалеру.
                Если точка пройдена (по старому прескалеру) то.

                     - Если есть новое значение пресклера, кладём его в
                     PSC . Мы всё равно ведь будем вызывать  UG ?? оно как
                     раз применится.
                     - Кладём ARR (рассчитанный по новому)
                     - Вызываем UG.
                 */
                else {
                    /*
                     Точка ещё не пройдена. В этом случае нельзя вызвать UG.
                     будет Glitch.
                     - пишем новый ARR  по старому прескалеру (без буфера)
                     - если PSC поменялся, то кладём уже по буферу и PSC и ARR.
                     */
                    TIM3->CR1 |= TIM_CR1_ARPE;
                    TIM3->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM3->CR1 &= ~TIM_CR1_ARPE;
                        TIM3->PSC = arr_with_calculations[1];
                        TIM3->ARR = arr_with_calculations[3];

                        // только для теста. TODO
                        TIM3->EGR = TIM_EGR_UG;
                    }

                    /* TIM3->CR1 |= TIM_CR1_ARPE;    // будем писать сразу в ARR (мимо shadow) */
                    /* TIM3->ARR = arr_with_calculations[2]; */
                    /* [> Костыльный предохранитель от убегания. <] */
                    /* if (TIM3->CNT > arr_with_calculations[2]) { */
                    /* TIM3->EGR |= TIM_EGR_UG; */
                    /* } */
                }
            }
            else {
                /*
                меньше. Скорость уменьшается. В этом случае обычно.период
                увеличивается. Но - прескалер может
                увеличиться (переключиться), и тем самым рассчётное значение ARR
                может быть значительно меньше, чем текущее.
                1.  Если не меняется  PSC - пишем  ARR без буфер. Это не
                вызывает никакх проблем вообще
                2.  �?зменение  PSC в этом случае пишем по старому прескелеру
                без буфера, рассчитываем новую пару и пишем по буферу!
                 */

                TIM3->CR1 |= TIM_CR1_ARPE;
                TIM3->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    /* TIM3->CR1 &= ~TIM_CR1_ARPE; */

                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numRL]->initial_tmp_flag = arr_with_calculations[1];
            whl_arr[numRL]->prev_speed = vRLrpm;
        }
    }

    ////////////    TIMER4 -  RR  ///////////////

    if (vRRrpm < 0x3F) {
        TIM4->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM4->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vRRrpm, arr_with_calculations, whl_arr, numRR);

        if (vRRrpm != whl_arr[numRR]->prev_speed) {
            if (vRRrpm > whl_arr[numRR]->prev_speed) {
                if (whl_arr[numRR]->psc_change_flag == 1) {
                    // this isn't functional now.
                    whl_arr[numRR]->psc_change_flag = 0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM4->CNT > (arr_with_calculations[2])) {
                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];
                    TIM4->EGR = TIM_EGR_UG;
                }
                /*
                В любом случае рассчитываем точку по старому прескалеру.
                Если точка пройдена (по старому прескалеру) то.

                     - Если есть новое значение пресклера, кладём его в
                     PSC . Мы всё равно ведь будем вызывать  UG ?? оно как
                     раз применится.
                     - Кладём ARR (рассчитанный по новому)
                     - Вызываем UG.
                 */
                else {
                    /*
                     Точка ещё не пройдена. В этом случае нельзя вызвать UG.
                     будет Glitch.
                     - пишем новый ARR  по старому прескалеру (без буфера)
                     - если PSC поменялся, то кладём уже по буферу и PSC и ARR.
                     */
                    TIM4->CR1 |= TIM_CR1_ARPE;
                    TIM4->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {
                        TIM4->CR1 &= ~TIM_CR1_ARPE;
                        TIM4->PSC = arr_with_calculations[1];
                        TIM4->ARR = arr_with_calculations[3];

                        // только для теста. TODO
                        TIM4->EGR = TIM_EGR_UG;
                    }

                    /* TIM4->CR1 |= TIM_CR1_ARPE;    // будем писать сразу в ARR (мимо shadow) */
                    /* TIM4->ARR = arr_with_calculations[2]; */
                    /* [> Костыльный предохранитель от убегания. <] */
                    /* if (TIM4->CNT > arr_with_calculations[2]) { */
                    /* TIM4->EGR |= TIM_EGR_UG; */
                    /* } */
                }
            }
            else {
                /*
                меньше. Скорость уменьшается. В этом случае обычно.период
                увеличивается. Но - прескалер может
                увеличиться (переключиться), и тем самым рассчётное значение ARR
                может быть значительно меньше, чем текущее.
                1.  Если не меняется  PSC - пишем  ARR без буфер. Это не
                вызывает никакх проблем вообще
                2.  �?зменение  PSC в этом случае пишем по старому прескелеру
                без буфера, рассчитываем новую пару и пишем по буферу!
                 */

                TIM4->CR1 |= TIM_CR1_ARPE;
                TIM4->ARR = arr_with_calculations[2];

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    /* TIM4->CR1 &= ~TIM_CR1_ARPE; */

                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];
                }
            }
            whl_arr[numRR]->initial_tmp_flag = arr_with_calculations[1];
            whl_arr[numRR]->prev_speed = vRRrpm;
        }
    }


    my_printf("exiting new speed settings, all set\n");
}



int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum)
{
    // Нужно будет отладить.

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
        /* arr[0] =  newpresc; */
        /* my_printf("prescaler_only_once for %d wheel\n\r", wheelnum); */
        whl_arr[wheelnum]->initial_tmp_flag = 1;
        arr[0] = 24;  // tmp
    }
    else {
        arr[0] = whl_arr[wheelnum]->prev_psc;
    }
    whl_arr[wheelnum]->prev_psc = newpresc;

    arr[1] = newpresc;

    uint32_t tmp;
    /* int tmp; */
    tmp = (APB1_CLK / (val * factor * MinutTeethFactor * (arr[0] + 1))) - 1;  // значение регистра ARR
    if (tmp > 65536) {
        /* my_printf(" -------------> GLITCH!\n\r"); */
        /* my_printf("res: %d \n\r", tmp); */
        /* my_printf("PSC:%d \n\r", arr[0]); */
        /* my_printf("signal: %d \n\n\r", val); */
        tmp = 65535;  // обрезаем. Хоть это и не правильно1
    }
    arr[2] = (int)tmp;
    if (arr[0] != arr[1]) {
        /* my_printf("             --              \n\r"); */
        /* my_printf("old_arr: %d\n\r", tmp); */

        tmp = ((APB1_CLK / (val * factor * MinutTeethFactor * (newpresc + 1))) - 1);  // значение регистра ARR
                                                                                      //
        /* my_printf("new_arr: %d\n\r", tmp); */

        if (tmp > 65536) {
            tmp = 65535;  // обрезаем. Хоть это и не правильно.
        }

        arr[3] = (int)tmp;
        /* arr[3] = 100; */

        /* my_printf("old_arr_casted: %d\n\r", arr[2]); */
        /* my_printf("arr_casted: %d\n\r", arr[3]); */
        /* my_printf("PSC old: %d\n\r", arr[0]); */
        /* my_printf("PSC current: %d\n\n\r", arr[1]); */
        /* my_printf("val: %d\n\n\r", val); */
        /* my_printf("\n"); */
    }
    else {
        arr[3] = arr[2];
    }

    return 0;
}

uint32_t calculete_period_only(int val)
{
    // For 32bit timer

    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;  // значение регистра ARR
}



