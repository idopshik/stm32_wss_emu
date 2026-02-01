/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define CAN_SPECIAL_ID 0x003
#define CAN_CMD_ID     0x004      // Новый ID для команд
#define APB1_CLK 150000000
#define MinutTeethFactor 1.6
#define numFL 0
#define numFR 1
#define numRL 2
#define numRR 3


extern system_state_t g_system_state;

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;

uint8_t canRX[8];  // CAN Bus Receive Buffer
uint8_t freshCanMsg = 0;
uint8_t freshCanCmd = 0;        // Новый флаг для команд
uint8_t canCmdData[8];          // Буфер для команд

char ms100Flag = 0;
char ms100Flag_2 = 0;

uint8_t recievingcounger = 0;    // for LED logic
uint8_t can_active_receiving = 0;

// Структура для режимов
typedef enum {
    MODE_NORMAL = 0,      // Рабочий режим RPM
    MODE_HI_Z = 1,        // Высокий импеданс (все выключено)
    MODE_TEST = 2,        // Тестовый режим
    MODE_BOOT = 3         // Режим загрузки
} system_mode_t;

// Структура состояния системы (улучшенная из новой версии)
typedef struct {
    system_mode_t current_mode;
    system_mode_t previous_mode;
    uint8_t channel_mask;           // Маска активных каналов (биты 0-3)
    uint8_t hi_impedance_active;    // Флаг Hi-Z режима
    uint32_t last_can_command_time;
    uint8_t pending_hi_z;
    uint8_t led_state;
    uint32_t led_last_toggle_time;
    uint8_t led_boot_flashed;
    uint32_t led_boot_start_time;
    uint8_t rpm_mode_active;
    uint8_t can_timeout_active;     // Таймаут CAN
} system_state_t;


// Структура для колеса (старая, но добавляем поля из новой версии)
typedef struct {
    uint8_t wheel_num;
    TIM_HandleTypeDef *htim;
    int prev_speed;
    int prev_psc;
    int cur_psc;
    int initial_tmp_flag;
    uint8_t psc_change_flag;
    
    // Новые поля из новой версии
    int target_psc;
    int target_arr;
    uint8_t pending_update;
    uint32_t prev_arr;
} whl_chnl;

// Массив структур колес (ДОЛЖЕН БЫТЬ ОБЪЯВЛЕН КАК ГЛОБАЛЬНЫЙ!)
whl_chnl *whl_arr[4];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);



/* USER CODE BEGIN PFP */

static void CANFD1_Set_Filtes(void);
void my_printf(const char *fmt, ...);
int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum);
uint32_t calculete_period_only(int val);
void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[]);

void system_init_modes(void);
void enter_hi_impedance_mode(void);
void exit_hi_impedance_mode(void);
void process_can_command(uint8_t *data);
void update_system_indicators(void);
void process_can_in_main(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++) ITM_SendChar(*ptr++);
    return len;
}

void PrintArrayLen(uint8_t *data_arr, uint8_t data_length)
{
    for (int i = 0; i < data_length; i++) {
        // printf("%lf\n",foo[i]);
        printf("%#x ", data_arr[i]);
    }
    printf("\n");
}

void PrintArray(uint8_t *data_arr)
{
    int debval;

    int denominator = sizeof(data_arr[0]);

    if (denominator > 0) {
        debval = (sizeof(data_arr) / denominator);
        for (int i = 0; i < debval; i++) {
            // printf("%lf\n",foo[i]);
            printf("%#x ", data_arr[i]);
        }
    }
    printf("\n");
}

uint32_t calculete_period_only(int val)
{
    // For 32bit timer

    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;  // значение регистра ARR
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
}


void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if (0 < vsprintf(string, fmt, argp))  // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string),
                          0xffffff);  // send message via UART
    }
}

void my_printf(const char *fmt, ...)  // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void print_test(void)
{
    printf("\r\nRTOS\r\nCharacters: %c %c\r\n", 'a', 65);
    printf("Decimals: %d %ld\r\n", 1977, 650000L);
    printf("Preceding with blanks: %10d\r\n", 1977);
    printf("Preceding with zeros: %010d\r\n", 1977);
    printf("Some different radices: %d %x %o %#x %#o\r\n", 100, 100, 100, 100, 100);
    printf("floats: %4.2f %+.0e %E\r\n", 3.1416, 3.1416, 3.1416);
    printf("Width trick: %*d\r\n", 5, 10);
    printf("%s\r\n\r\n", "A string");

    my_printf("\r\nRTOS\r\nCharacters: %c %c\r\n", 'a', 65);
    my_printf("Decimals: %d %ld\r\n", 1977, 650000L);
    my_printf("Preceding with blanks: %10d\r\n", 1977);
    my_printf("Preceding with zeros: %010d\r\n", 1977);
    my_printf("Some different radices: %d %x %o %#x %#o\r\n", 100, 100, 100, 100, 100);
    my_printf("floats: %4.2f %+.0e %E\r\n", 3.1416, 3.1416, 3.1416);
    my_printf("Width trick: %*d\r\n", 5, 10);
    my_printf("%s\r\n\r\n", "A string");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        /*my_printf("100 ms \n");*/
        ms100Flag = 1;
        ms100Flag_2 = 1;
    }
    if (htim->Instance == TIM8) {
        // has to be 70 ms. to flash LED ~8 times a sec

        if (can_active_receiving == 1){
            // toggle LED
            
            HAL_GPIO_TogglePin(GPIOA, EXT_LED_Pin);

            recievingcounger -= 1;

            if (recievingcounger < 1){
                can_active_receiving = 0;
                // deactivate LED
                HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            }

        }
        
    }

    if (htim->Instance == TIM1) {
        HAL_GPIO_TogglePin(GPIOA, tim1_out_Pin);
    }
    else if (htim->Instance == TIM3) {
        HAL_GPIO_TogglePin(GPIOA, tim3_out_Pin);
    }
    else if (htim->Instance == TIM4) {
        HAL_GPIO_TogglePin(GPIOB, tim4_out_Pin);
    }
}

void calculate_arr(whl_chnl *whl_arr[], int whl)
{
    //
    // примеры обращения к элемента структуры
    //   whl_arr[whl]->pref_speed
    //   whl_arr[whl]->cur_psc
    //   whl_arr[whl]->flag
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    CANFD1_Set_Filtes();

    // Инициализация системы режимов
    system_init_modes();

    // Start four timers (как в старом коде)
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim8);

    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

#ifdef DEBUG
    printf("[ INFO ] Program start now\n");
#endif

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    
    // Инициализация структур колес (старый код, но используем обновленную структуру)
    whl_chnl fl_whl_s = {numFL, &htim1, 0, 24, 0, 0, 0, 0, 0, 0};
    whl_chnl fr_whl_s = {numFR, &htim2, 0, 12, 0, 0, 0, 0, 0, 0};
    whl_chnl rl_whl_s = {numRL, &htim3, 0, 24, 0, 0, 0, 0, 0, 0};
    whl_chnl rr_whl_s = {numRR, &htim4, 0, 24, 0, 0, 0, 0, 0, 0};

    whl_arr[numFL] = &fl_whl_s;
    whl_arr[numFR] = &fr_whl_s;
    whl_arr[numRL] = &rl_whl_s;
    whl_arr[numRR] = &rr_whl_s;

    set_new_speeds(0,0,0,0, whl_arr); // initialy - no output

    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);

    // Переход в Hi-Z режим после инициализации
    enter_hi_impedance_mode();
    
    while (1) {
        // Обработка CAN сообщений
        process_can_in_main();
        
        // Обновление индикаторов и проверка таймаутов
        update_system_indicators();
        
        // Старая логика мигания светодиодов (100 мс)
        if (ms100Flag > 0) {
            ms100Flag = 0;
            HAL_GPIO_TogglePin(GPIOB, Out_1_Pin);
        }
        
        // Небольшая задержка для снижения нагрузки на CPU
        HAL_Delay(1);
    }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 7;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 12;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 600000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 15000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 624;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin|tim3_out_Pin|tim1_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Out_1_Pin|Out_2_Pin|tim4_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_LED_Pin tim3_out_Pin tim1_out_Pin */
  GPIO_InitStruct.Pin = EXT_LED_Pin|tim3_out_Pin|tim1_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Out_1_Pin Out_2_Pin tim4_out_Pin */
  GPIO_InitStruct.Pin = Out_1_Pin|Out_2_Pin|tim4_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// Инициализация системы режимов
void system_init_modes(void)
{
    memset(&g_system_state, 0, sizeof(g_system_state));
    g_system_state.current_mode = MODE_BOOT;
    g_system_state.led_boot_start_time = HAL_GetTick();
    g_system_state.last_can_command_time = HAL_GetTick();
    
    my_printf("[SYS] Mode system initialized\n");
}

// Вход в Hi-Z режим
void enter_hi_impedance_mode(void)
{
    my_printf("[MODE] Entering Hi-Z\n");
    
    // 1. Выключаем все таймеры
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // 2. Сбрасываем скорости в структурах колес
    for(int i = 0; i < 4; i++) {
        if(whl_arr[i] != NULL) {
            whl_arr[i]->prev_speed = 0;
        }
    }
    
    // 3. Обновляем состояние
    g_system_state.previous_mode = g_system_state.current_mode;
    g_system_state.current_mode = MODE_HI_Z;
    g_system_state.hi_impedance_active = 1;
    g_system_state.channel_mask = 0;
    
    // 4. Выключаем светодиод
    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
}

// Выход из Hi-Z режима
void exit_hi_impedance_mode(void)
{
    my_printf("[MODE] Exiting Hi-Z\n");
    g_system_state.current_mode = MODE_NORMAL;
    g_system_state.hi_impedance_active = 0;
    // Таймеры включатся при получении RPM данных
}

// Обработка CAN команд
void process_can_command(uint8_t *data)
{
    uint8_t command = data[0];
    
    switch(command) {
        case 0x01:  // Войти в Hi-Z
            enter_hi_impedance_mode();
            break;
            
        case 0x02:  // Выйти из Hi-Z
            exit_hi_impedance_mode();
            break;
            
        case 0x03:  // Тестовый режим
            if(g_system_state.current_mode != MODE_HI_Z) {
                int rpm = (data[1] << 8) | data[2];
                set_new_speeds(rpm, rpm, rpm, rpm, whl_arr);
                my_printf("[TEST] Fixed RPM: %d\n", rpm);
                g_system_state.current_mode = MODE_TEST;
            }
            break;
            
        case 0x04:  // Включить/выключить каналы
            {
                uint8_t mask = data[1];
                g_system_state.channel_mask = mask & 0x0F;  // Только 4 бита
                my_printf("[CMD] Channel mask: 0x%02X\n", g_system_state.channel_mask);
            }
            break;
            
        case 0x05:  // Вернуться в нормальный режим
            g_system_state.current_mode = MODE_NORMAL;
            my_printf("[CMD] Normal mode\n");
            break;
            
        default:
            my_printf("[CMD] Unknown: 0x%02X\n", command);
            break;
    }
    
    g_system_state.last_can_command_time = HAL_GetTick();
}

// Обработка CAN в основном цикле
void process_can_in_main(void)
{
    // Обработка RPM данных
    if (freshCanMsg == 1) {
        freshCanMsg = 0;
        
        if(g_system_state.current_mode == MODE_NORMAL || 
           g_system_state.current_mode == MODE_TEST) {
            // Парсим RPM данные
            int vFLrpm = (uint8_t)canRX[0] << 8 | (uint8_t)canRX[1];
            int vFRrpm = (uint8_t)canRX[2] << 8 | (uint8_t)canRX[3];
            int vRLrpm = (uint8_t)canRX[4] << 8 | (uint8_t)canRX[5];
            int vRRrpm = (uint8_t)canRX[6] << 8 | (uint8_t)canRX[7];
            
            set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);
            
            g_system_state.rpm_mode_active = 1;
            g_system_state.last_can_command_time = HAL_GetTick();
        }
        
        // Индикация приема
        can_active_receiving = 1;
        recievingcounger = 4;
    }
    
    // Обработка команд
    if (freshCanCmd == 1) {
        freshCanCmd = 0;
        process_can_command(canCmdData);
    }
}

// Обновление индикаторов
void update_system_indicators(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(current_time - last_update < 10) {
        return;
    }
    last_update = current_time;
    
    // Таймаут: если долго нет команд - переходим в Hi-Z
    if(g_system_state.current_mode == MODE_NORMAL) {
        if(current_time - g_system_state.last_can_command_time > 5000) { // 5 секунд
            my_printf("[TIMEOUT] No CAN data for 5s\n");
            enter_hi_impedance_mode();
        }
    }
}


// Инициализация системы
void system_init(void)
{
    sys_state.current_mode = MODE_BOOT;
    sys_state.channel_enabled = 0;
    sys_state.last_can_time = HAL_GetTick();
    sys_state.led_blink = 0;
    
    my_printf("[SYS] Initialized - Boot mode\n");
}

// Вход в Hi-Z режим
void enter_hi_z_mode(void)
{
    my_printf("[MODE] Entering Hi-Z\n");
    
    // 1. Выключаем все таймеры
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // 2. Сбрасываем все скорости в структурах
    for(int i = 0; i < 4; i++) {
        if(whl_arr[i] != NULL) {
            whl_arr[i]->prev_speed = 0;
        }
    }
    
    // 3. Обновляем состояние
    sys_state.current_mode = MODE_HI_Z;
    sys_state.channel_enabled = 0;
    
    // 4. Светодиодная индикация
    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
}

// Выход из Hi-Z режима
void exit_hi_z_mode(void)
{
    my_printf("[MODE] Exiting Hi-Z\n");
    sys_state.current_mode = MODE_NORMAL;
    // Таймеры включатся при первом RPM сообщении
}


static void CANFD1_Set_Filtes(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // Фильтр 0: Принимаем ID 0x003 (RPM данные)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;  // ID 0x003
    sFilterConfig.FilterID2 = 0x07FF;          // Маска
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // Фильтр 1: Принимаем ID 0x004 (команды управления)
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = CAN_CMD_ID;      // ID 0x004
    sFilterConfig.FilterID2 = 0x07FF;          // Маска
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Глобальный фильтр
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_REJECT,          // Стандартные ID не прошедшие фильтр
        FDCAN_REJECT,          // Расширенные ID не прошедшие фильтр
        FDCAN_FILTER_REMOTE,   // Удаленные фреймы с фильтром
        FDCAN_FILTER_REMOTE)   // Удаленные фреймы без фильтра
        != HAL_OK) {
        Error_Handler();
    }

    // Активируем прерывание
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    
    // Запускаем FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    my_printf("CAN Filter: ID 0x003 & 0x004\n");
}

// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            Error_Handler();
        }
        else {
            if (RxHeader1.Identifier == CAN_SPECIAL_ID) {
                // RPM данные
                freshCanMsg = 1;
            }
            else if (RxHeader1.Identifier == CAN_CMD_ID) {
                // Команды управления - копируем в буфер команд
                memcpy(canCmdData, canRX, 8);
                freshCanCmd = 1;
            }
            else {
                my_printf("wrongID: %#x \n\r", RxHeader1.Identifier);
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
