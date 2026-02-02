#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include "main.h"

// Константы
#define APB1_CLK 150000000
#define MinutTeethFactor 1.6

// Структура колеса
typedef struct {
    uint32_t prev_speed;      // предыдущая скорость
    uint16_t prev_psc;        // предыдущий предделитель
    uint8_t initial_tmp_flag; // флаг инициализации
    uint8_t psc_change_flag;  // флаг изменения предделителя
} whl_chnl;

// Глобальный массив указателей на структуры колес
//
extern whl_chnl *whl_arr[4];


// Прототипы функций
void wheel_control_init(void);
void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm);
int calculete_prsc_and_perio(int val, int *arr, int wheelnum);
uint32_t calculete_period_only(int val);

#endif
