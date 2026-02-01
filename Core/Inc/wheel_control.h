#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include "main.h"

#define numFL 0
#define numFR 1
#define numRL 2
#define numRR 3
#define APB1_CLK 150000000
#define MinutTeethFactor 1.6

typedef struct {
    uint8_t wheel_num;
    TIM_HandleTypeDef *htim;
    int prev_speed;
    int prev_psc;
    int cur_psc;
    int initial_tmp_flag;
    uint8_t psc_change_flag;
} whl_chnl;

// Объявляем глобальный массив
extern whl_chnl *whl_arr[4];

// Функции как в оригинале:
void wheel_control_init(void);
void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[]);
int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum);
uint32_t calculete_period_only(int val);

#endif
