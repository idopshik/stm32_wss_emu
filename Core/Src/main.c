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

#include "system_modes.h"
#include "can_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN_SPECIAL_ID 0x003
#define APB1_CLK 150000000
#define MinutTeethFactor 1.6
#define numFL 0
#define numFR 1
#define numRL 2
#define numRR 3

#define TOTAL_DIVIDER_CONST 2343750000ULL

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;

uint8_t canRX[8];
uint8_t freshCanMsg = 0;

char ms100Flag = 0;
char ms100Flag_2 = 0;

uint8_t recievingcounger = 0;
uint8_t can_active_receiving = 0;

typedef struct {
    uint8_t wheel_num;
    TIM_HandleTypeDef *htim;
    int prev_speed;
    int prev_psc;
    int target_psc;
    int target_arr;
    uint8_t pending_update;
    uint32_t prev_arr;
    uint8_t initial_tmp_flag;
    uint8_t psc_change_flag;
} whl_chnl;

whl_chnl *whl_arr[4];


volatile uint32_t tim4_irq_counter = 0;
// Структура для диагностики
typedef struct {
    uint32_t last_cnt;
    uint32_t stuck_count;
    uint32_t last_irq_time;
    uint8_t is_running;
} TimerDebug_t;
TimerDebug_t tim4_debug = {0};



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
volatile uint8_t can_rx_pending = 0;
can_msg_t can_rx_msg;

volatile uint8_t can_tx_status_pending = 0;
volatile uint8_t can_tx_error_pending = 0;
uint8_t can_tx_error_code = 0;

uint8_t update_led_flag = 0;
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
void update_wheel_seamless(TIM_TypeDef* TIMx, int rpm, whl_chnl* wheel);
void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[]);
void print_timer_status(void);
void update_system_indicators(void);
void check_system_health(void);
void test_rpm_calculation(void);
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

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if (0 < vsprintf(string, fmt, argp))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), 0xffffff);
    }
}

void my_printf(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

uint32_t calculete_period_only(int val)
{
    if (val <= 0) {
        val = 1;
        /* printf("[CALC_PERIOD_TIM2] WARNING: Zero or negative input, using 1\n"); */
    }

    if (val > 65535) {
        val = 65535;
        /* printf("[CALC_PERIOD_TIM2] WARNING: Input too large, clamped to 65535\n"); */
    }

    uint64_t total_divider = TOTAL_DIVIDER_CONST / val;
    uint32_t arr = (uint32_t)(total_divider / 13) - 1;

    /* printf("[CALC_PERIOD_TIM2] Input RPM: %d\n", val); */
    /* printf("[CALC_PERIOD_TIM2] Total Divider: %lu\n", (uint32_t)total_divider); */
    /* printf("[CALC_PERIOD_TIM2] PSC: 12\n"); */
    /* printf("[CALC_PERIOD_TIM2] ARR: %lu\n", arr); */

    return arr;
}

int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum)
{
#ifdef DEBUG
    my_printf("[EDGE_CASE] ===== INPUT ANALYSIS =====\n");
    my_printf("[EDGE_CASE] Raw Input RPM: %d\n", val);
    my_printf("[EDGE_CASE] Wheel Number: %d\n", wheelnum);
#endif

    if (val <= 0) {
#ifdef DEBUG
        my_printf("[EDGE_CASE] WARNING: Zero/Negative input. Clamping to 1!\n");
#endif
        val = 1;
    }

    if (val > 65535) {
#ifdef DEBUG
        my_printf("[EDGE_CASE] WARNING: Input exceeds 16-bit range. Clamping to 65535!\n");
#endif
        val = 65535;
    }

    int newpresc;
    if (val < 4274) {
        newpresc = 1200;
#ifdef DEBUG
        my_printf("[EDGE_CASE] PSC selection: 1200 (RPM < 4274)\n");
#endif
    } else {
        newpresc = 24;
#ifdef DEBUG
        my_printf("[EDGE_CASE] PSC selection: 24 (RPM >= 4274)\n");
#endif
    }

    uint64_t total_divider = TOTAL_DIVIDER_CONST / val;
    uint32_t tmp = (uint32_t)(total_divider / (newpresc + 1)) - 1;

    if (tmp > 65535) {
#ifdef DEBUG
        my_printf("[EDGE_CASE] WARNING: ARR exceeds 16-bit range. Clamping to 65535!\n");
#endif
        tmp = 65535;
    }

#ifdef DEBUG
    my_printf("[EDGE_CASE] Total Divider: %lu\n", (uint32_t)total_divider);
    my_printf("[EDGE_CASE] Selected PSC: %d\n", newpresc);
    my_printf("[EDGE_CASE] Calculated ARR: %lu\n", tmp);
    my_printf("[EDGE_CASE] ===== END ANALYSIS =====\n\n");
#endif

    arr[0] = whl_arr[wheelnum]->prev_psc;
    arr[1] = newpresc;
    arr[2] = (int)tmp;
    arr[3] = (int)tmp;
    whl_arr[wheelnum]->prev_psc = newpresc;

    return 0;
}

void update_wheel_seamless(TIM_TypeDef* TIMx, int rpm, whl_chnl* wheel)
{
    // ВАЖНО: Всегда обновляем prev_speed первым делом!
    wheel->prev_speed = rpm;
    
    // Если RPM < 63 - ВЫКЛЮЧАЕМ таймер и выходим
    if (rpm < 0x3F) {
        TIMx->CR1 &= ~TIM_CR1_CEN;  // Выключаем таймер
        wheel->pending_update = 0;
        
        uint8_t channel_bit = 1 << wheel->wheel_num;
        g_system_state.channel_mask &= ~channel_bit;
        return;
    }
    
    // Включаем канал в маске
    uint8_t channel_bit = 1 << wheel->wheel_num;
    g_system_state.channel_mask |= channel_bit;
    
    // Расчет параметров
    int calc[4];
    calculete_prsc_and_perio(rpm, calc, whl_arr, wheel->wheel_num);
    
    int new_psc = calc[1];
    int new_arr = calc[3];
    
    wheel->prev_arr = TIMx->ARR;
    
    // Если PSC не меняется - просто обновляем ARR
    if (new_psc == wheel->prev_psc) {
        // Отключаем ARPE для прямой записи
        uint32_t cr1_temp = TIMx->CR1;
        TIMx->CR1 &= ~TIM_CR1_ARPE;
        
        // Записываем ARR напрямую
        TIMx->ARR = new_arr;
        
        // Включаем ARPE обратно
        TIMx->CR1 = cr1_temp | TIM_CR1_ARPE;
        
        // Принудительный update для загрузки ARR
        TIMx->EGR |= TIM_EGR_UG;
        
        wheel->prev_psc = new_psc;
        wheel->pending_update = 0;
        
        // ГАРАНТИРОВАННО включаем таймер
        TIMx->CR1 |= TIM_CR1_CEN;
        return;
    }
    
    // Если нужно менять PSC - ставим флаг ожидания
    wheel->target_psc = new_psc;
    wheel->target_arr = new_arr;
    wheel->pending_update = 1;
    
    // Включаем прерывание для seamless переключения
    __HAL_TIM_ENABLE_IT(wheel->htim, TIM_IT_UPDATE);
    
    // ГАРАНТИРОВАННО включаем таймер (если RPM >= 63)
    TIMx->CR1 |= TIM_CR1_CEN;
}

void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[]) 
{
    // Только краткая информация через SWV
    printf("[SET] %d %d %d %d\n", vFLrpm, vFRrpm, vRLrpm, vRRrpm);
    printf("[DBG] FL: new=%d prev=%d\n", vFLrpm, whl_arr[numFL]->prev_speed);

    // FL wheel (TIM1)
    if (vFLrpm != whl_arr[numFL]->prev_speed) {
        update_wheel_seamless(TIM1, vFLrpm, whl_arr[numFL]);
    }
    
    // FR wheel (TIM2) - ОСОБАЯ ОБРАБОТКА
    if (vFRrpm != whl_arr[numFR]->prev_speed) 
    {
        printf("[TIM2] RPM change: %d -> %d\n", whl_arr[numFR]->prev_speed, vFRrpm);
        
        if (vFRrpm < 0x3F) {
            // Остановка таймера
            printf("[TIM2] Stopping (RPM=%d < 63)\n", vFRrpm);
            uint32_t cr1_before = TIM2->CR1;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            printf("[TIM2] CEN: %d -> %d\n", 
                   (cr1_before & TIM_CR1_CEN) ? 1 : 0,
                   (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0);
            g_system_state.channel_mask &= ~(1 << 1);
        } 
        else 
        {
            printf("[TIM2] Starting/updating (RPM=%d)\n", vFRrpm);
            
            // 1. Проверка и включение тактирования
            uint32_t apb1enr1 = RCC->APB1ENR1;
            if (!(apb1enr1 & RCC_APB1ENR1_TIM2EN)) {
                printf("[TIM2_WARN] Clock was disabled, enabling\n");
                __HAL_RCC_TIM2_CLK_ENABLE();
                // Задержка для стабилизации
                for(volatile int i=0; i<1000; i++);
            }
            
            // 2. БЕЗОПАСНОЕ чтение CR1 - проверяем адрес
            volatile uint32_t* tim2_cr1_ptr = &TIM2->CR1;
            if ((uint32_t)tim2_cr1_ptr < 0x40000000 || (uint32_t)tim2_cr1_ptr > 0x60000000) {
                printf("[TIM2_ERROR] Invalid TIM2 pointer!\n");
                return;
            }
            
            uint32_t cr1_before = TIM2->CR1;
            printf("[TIM2] CR1 read: 0x%04lX\n", cr1_before);
            
            // 3. Включение таймера
            TIM2->CR1 = cr1_before | TIM_CR1_CEN;
            
            uint32_t cr1_after = TIM2->CR1;
            printf("[TIM2] CEN set: %d -> %d\n",
                   (cr1_before & TIM_CR1_CEN) ? 1 : 0,
                   (cr1_after & TIM_CR1_CEN) ? 1 : 0);
            
            g_system_state.channel_mask |= (1 << 1);
            
            // 4. Расчет периода
            if (vFRrpm <= 0) {
                printf("[TIM2_ERROR] Invalid RPM=%d\n", vFRrpm);
                whl_arr[numFR]->prev_speed = vFRrpm;
                return;
            }
            
            uint32_t period = calculete_period_only(vFRrpm);
            printf("[TIM2] Period calculated: %lu\n", period);
            
            // 5. БЕЗОПАСНАЯ запись ARR - пошагово
            printf("[TIM2] Step 1: Save CR1\n");
            uint32_t saved_cr1 = TIM2->CR1;
            
            printf("[TIM2] Step 2: Disable ARPE\n");
            TIM2->CR1 = saved_cr1 & ~TIM_CR1_ARPE;
            
            printf("[TIM2] Step 3: Write ARR\n");
            TIM2->ARR = period;
            
            printf("[TIM2] Step 4: Restore CR1 with ARPE\n");
            TIM2->CR1 = saved_cr1 | TIM_CR1_ARPE;
            
            printf("[TIM2] Step 5: Generate UG\n");
            /* TIM2->EGR |= TIM_EGR_UG; */
            
            // 6. ФИНАЛЬНАЯ проверка - отдельными printf
            printf("[TIM2] Step 6: Verify\n");
            
            uint32_t final_cr1 = TIM2->CR1;
            printf("[TIM2] Final CR1: 0x%04lX\n", final_cr1);
            
            uint32_t final_arr = TIM2->ARR;
            printf("[TIM2] Final ARR: %lu\n", final_arr);
            
            uint32_t final_psc = TIM2->PSC;
            printf("[TIM2] Final PSC: %lu\n", final_psc);
            
            printf("[TIM2] Setup complete\n");
        }
        
        whl_arr[numFR]->prev_speed = vFRrpm;
    }
    
    // RL wheel (TIM3)
    if (vRLrpm != whl_arr[numRL]->prev_speed) {
        update_wheel_seamless(TIM3, vRLrpm, whl_arr[numRL]);
    }
    
    // RR wheel (TIM4)
    if (vRRrpm != whl_arr[numRR]->prev_speed) {
        update_wheel_seamless(TIM4, vRRrpm, whl_arr[numRR]);
    }
    
    printf("[SET] Done\n");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for(int i = 0; i < 4; i++) {
        if(whl_arr[i] != NULL && 
           whl_arr[i]->pending_update && 
           whl_arr[i]->htim->Instance == htim->Instance) {
            
            htim->Instance->PSC = whl_arr[i]->target_psc;
            htim->Instance->ARR = whl_arr[i]->target_arr;
            
            whl_arr[i]->prev_psc = whl_arr[i]->target_psc;
            whl_arr[i]->pending_update = 0;
            
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
            
#ifdef DEBUG
            my_printf("PSC seamless switch: TIM%c, PSC=%d, ARR=%d\n", 
                   '1' + i, whl_arr[i]->target_psc, whl_arr[i]->target_arr);
#endif
            break;
        }
    }
    
    if (htim->Instance == TIM6) {
        ms100Flag = 1;
        ms100Flag_2 = 1;
    }
    
    if (htim->Instance == TIM8) {
        if (can_active_receiving == 1){
            HAL_GPIO_TogglePin(GPIOA, EXT_LED_Pin);
            recievingcounger -= 1;
            if (recievingcounger < 1){
                can_active_receiving = 0;
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
        
        tim4_irq_counter++;  // Теперь переменная объявлена
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
        
        // Логируем каждое 300-е прерывание
        if (tim4_irq_counter % 300 == 0) {
            printf("[IRQ TIM4] #%lu\n", tim4_irq_counter);
        }
    }
}

void test_rpm_calculation(void)
{
#ifdef DEBUG
    my_printf("\n=== TEST RPM CALCULATION ===\n");
    
    int test_val = 1000;
    float f_signal = test_val * 0.032f;
    float f_timer = test_val * 0.064f;
    uint64_t total_divider = TOTAL_DIVIDER_CONST / test_val;
    
    uint32_t arr_tim2 = (uint32_t)(total_divider / 13) - 1;
    
    int psc_16bit = (test_val < 4274) ? 1200 : 24;
    uint32_t arr_16bit = (uint32_t)(total_divider / (psc_16bit + 1)) - 1;
    
    my_printf("Test val=%d:\n", test_val);
    my_printf("  f_signal=%.1fHz, f_timer=%.1fHz\n", f_signal, f_timer);
    my_printf("  Total_Divider=%llu\n", total_divider);
    my_printf("  TIM2: PSC=12, ARR=%lu\n", arr_tim2);
    my_printf("  TIM1/3/4: PSC=%d, ARR=%lu\n", psc_16bit, arr_16bit);
    
    test_val = 4274;
    total_divider = TOTAL_DIVIDER_CONST / test_val;
    psc_16bit = (test_val < 4274) ? 1200 : 24;
    
    my_printf("\nBorder val=%d:\n", test_val);
    my_printf("  Total_Divider=%llu\n", total_divider);
    my_printf("  PSC should be=%d (>=4274 → 24)\n", psc_16bit);
#endif
}


void diagnostic_tim4_status(void) {
    static uint32_t last_debug2 = 0;
    
    if (HAL_GetTick() - last_debug2 > 3000) {
        last_debug2 = HAL_GetTick();
        
        my_printf("\n=== TIM4 DIAG [Mode: %s] ===\n", 
                  get_mode_name(g_system_state.current_mode));
        
        // 1. Clock enable
        my_printf("RCC->APB1ENR1: 0x%08lX ", RCC->APB1ENR1);
        my_printf("TIM4_EN=%lu\n", (RCC->APB1ENR1 & RCC_APB1ENR1_TIM4EN) ? 1 : 0);
        
        // 2. Control register
        my_printf("CR1: 0x%04lX ", TIM4->CR1);
        my_printf("CEN=%lu URS=%lu DIR=%lu\n",
               (TIM4->CR1 & TIM_CR1_CEN) ? 1 : 0,
               (TIM4->CR1 & TIM_CR1_URS) ? 1 : 0,
               (TIM4->CR1 & TIM_CR1_DIR) ? 1 : 0);
        
        // 3. Parameters
        my_printf("PSC: %lu ARR: %lu CNT: %lu\n", 
                  TIM4->PSC, TIM4->ARR, TIM4->CNT);
        
        // 4. Status
        my_printf("SR: 0x%04lX UIF=%lu\n", TIM4->SR, (TIM4->SR & 1));
        
        // 5. Interrupts
        my_printf("DIER: 0x%04lX UIE=%lu\n", TIM4->DIER, (TIM4->DIER & 1));
        my_printf("IRQ count: %lu\n", tim4_irq_counter);

        // Полная диагностика GPIO
        uint32_t moder = GPIOB->MODER;
        uint32_t otyper = GPIOB->OTYPER;
        uint32_t pupdr = GPIOB->PUPDR;
        
        my_printf("MODER[PB9]: %lu (0=Input, 1=Output, 2=Alt, 3=Analog)\n", 
                  (moder >> 18) & 0x03);
        my_printf("OTYPER[PB9]: %lu (0=Push-pull, 1=Open-drain)\n", 
                  (otyper >> 9) & 0x01);
        my_printf("PUPDR[PB9]: %lu (0=No pull, 1=Pull-up, 2=Pull-down)\n", 
                  (pupdr >> 18) & 0x03);
        }
}

void diagnostic_tim2_status(void) {
    static uint32_t last_debug2 = 0;
    
    if (HAL_GetTick() - last_debug2 > 3000) {
        last_debug2 = HAL_GetTick();
        
        my_printf("\n=== TIM2 DIAG [Mode: %s] ===\n", 
                  get_mode_name(g_system_state.current_mode));
        
        // 1. Clock enable
        my_printf("RCC->APB1ENR1: 0x%08lX ", RCC->APB1ENR1);
        my_printf("TIM2_EN=%lu\n", (RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) ? 1 : 0);
        
        // 2. Control register
        my_printf("CR1: 0x%04lX ", TIM2->CR1);
        my_printf("CEN=%lu URS=%lu DIR=%lu\n",
               (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0,
               (TIM2->CR1 & TIM_CR1_URS) ? 1 : 0,
               (TIM2->CR1 & TIM_CR1_DIR) ? 1 : 0);
        
        // 3. Parameters
        my_printf("PSC: %lu ARR: %lu CNT: %lu\n", 
                  TIM2->PSC, TIM2->ARR, TIM2->CNT);
        
        // 4. Status
        my_printf("SR: 0x%04lX UIF=%lu\n", TIM2->SR, (TIM2->SR & 1));
        
        // 5. Interrupts
        my_printf("DIER: 0x%04lX UIE=%lu\n", TIM2->DIER, (TIM2->DIER & 1));
    }
}


void diagnostic_mode_status(void) {
    static uint32_t last_mode_debug = 0;
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_mode_debug > 3000) {
        last_mode_debug = current_time;
        
        my_printf("\n=== CURRENT MODE STATUS ===\n");
        my_printf("Mode: %s\n", get_mode_name(g_system_state.current_mode));
        my_printf("Channel Mask: 0x%02X\n", g_system_state.channel_mask);
        my_printf("Hi-Z Active: %s\n", 
                  g_system_state.hi_impedance_active ? "YES" : "NO");
        my_printf("Last CAN Command: %lu ms ago\n", 
                  current_time - g_system_state.last_can_command_time);
        my_printf("Uptime: %lu seconds\n", system_get_uptime_seconds());
        my_printf("=======================\n");
    }
}


void diagnostic_gpio_status(void) {
    static uint32_t last_gpio_debug = 0;
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_gpio_debug > 3000) {
        last_gpio_debug = current_time;
        
        my_printf("\n=== GPIO STATUS ===\n");
        
        // PA8 (FL)
        my_printf("PA8: MODER=%lu, IDR=%lu\n", 
                  (GPIOA->MODER >> 16) & 0x03, 
                  (GPIOA->IDR >> 8) & 0x01);
        
        // PA15 (FR)
        my_printf("PA15: MODER=%lu, IDR=%lu\n", 
                  (GPIOA->MODER >> 30) & 0x03, 
                  (GPIOA->IDR >> 15) & 0x01);
        
        // PA6 (RL)
        my_printf("PA6: MODER=%lu, IDR=%lu\n", 
                  (GPIOA->MODER >> 12) & 0x03, 
                  (GPIOA->IDR >> 6) & 0x01);
        
        // PB6 (RR)
        my_printf("PB6: MODER=%lu, IDR=%lu\n", 
                  (GPIOB->MODER >> 12) & 0x03, 
                  (GPIOB->IDR >> 6) & 0x01);
        
        // PB9
        my_printf("PB9: MODER=%lu, IDR=%lu\n", 
                  (GPIOB->MODER >> 18) & 0x03, 
                  (GPIOB->IDR >> 9) & 0x01);
    }
}




void diagnostic_alternative_functions(void) {
    static uint32_t last_af_debug = 0;
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_af_debug > 3000) {
        last_af_debug = current_time;
        
        my_printf("\n=== ALTERNATIVE FUNCTIONS STATUS ===\n");
        
        // PA8 (TIM1_CH1)
        my_printf("PA8 AF: %lu (AFRL=%08lX)\n", 
                  (GPIOA->AFR[0] >> 0) & 0x0F, 
                  GPIOA->AFR[0]);
        
        // PA15 (TIM2_CH1)
        my_printf("PA15 AF: %lu (AFRH=%08lX)\n", 
                  (GPIOA->AFR[1] >> 28) & 0x0F, 
                  GPIOA->AFR[1]);
        
        // PA6 (TIM3_CH1)
        my_printf("PA6 AF: %lu (AFRL=%08lX)\n", 
                  (GPIOA->AFR[0] >> 24) & 0x0F, 
                  GPIOA->AFR[0]);
        
        // Проверка SYSCFG
        my_printf("EXTI PA15 line: %lu\n", 
                  (SYSCFG->EXTICR[3] >> 12) & 0x0F);
    }
}

// Добавь эту проверку:
void check_timer_interrupts(void)
{
    static uint32_t last_af_debug4 = 0;
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_af_debug4 > 3000) {
        last_af_debug4 = current_time;
        my_printf("\n=== TIMER INTERRUPTS CHECK ===\n");
        my_printf("TIM1 DIER: 0x%04X (UIE=%d)\n", TIM1->DIER, (TIM1->DIER & TIM_DIER_UIE) ? 1 : 0);
        my_printf("TIM2 DIER: 0x%04X (UIE=%d)\n", TIM2->DIER, (TIM2->DIER & TIM_DIER_UIE) ? 1 : 0);
        my_printf("TIM3 DIER: 0x%04X (UIE=%d)\n", TIM3->DIER, (TIM3->DIER & TIM_DIER_UIE) ? 1 : 0);
        my_printf("TIM4 DIER: 0x%04X (UIE=%d)\n", TIM4->DIER, (TIM4->DIER & TIM_DIER_UIE) ? 1 : 0);
        
        // Проверь NVIC
        my_printf("NVIC TIM1_UP: %s\n", NVIC_GetEnableIRQ(TIM1_UP_TIM16_IRQn) ? "ON" : "OFF");
        my_printf("NVIC TIM2: %s\n", NVIC_GetEnableIRQ(TIM2_IRQn) ? "ON" : "OFF");
        my_printf("NVIC TIM3: %s\n", NVIC_GetEnableIRQ(TIM3_IRQn) ? "ON" : "OFF");
        my_printf("NVIC TIM4: %s\n", NVIC_GetEnableIRQ(TIM4_IRQn) ? "ON" : "OFF");
        my_printf("==============================\n");
    }
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

  system_init_modes();

  HAL_Delay(1500);
  enter_hi_impedance_mode();
  diagnostic_mode_status();


  CANFD1_Set_Filtes();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim8);

  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

#ifdef DEBUG
  my_printf("[ INFO ] Program start now\n");
#endif

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

#ifdef DEBUG
  my_printf("Seamless PSC switching enabled\n");
#endif

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM8_CC_IRQn, 5, 0);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 6, 0);
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 10, 0);
  HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn, 10, 0);
  HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);


  printf("\n\n=== SWV TEST ===\n");
  printf("Если видишь это - SWV работает!\n");
  printf("TIM4->CR1 = 0x%04lX\n", TIM4->CR1);

#ifdef DEBUG
  my_printf("\n========================================\n");
  my_printf("SYSTEM STARTED SUCCESSFULLY\n");
  my_printf("Current mode: %s\n", get_mode_name(g_system_state.current_mode));
  my_printf("No EEPROM - all settings volatile\n");
  my_printf("========================================\n\n");

  my_printf("=== Clock Configuration ===\n");
  my_printf("SystemCoreClock: %lu Hz\n", SystemCoreClock);
  my_printf("HCLK: %lu Hz\n", HAL_RCC_GetHCLKFreq());
  my_printf("PCLK1: %lu Hz\n", HAL_RCC_GetPCLK1Freq());
  my_printf("PCLK2: %lu Hz\n", HAL_RCC_GetPCLK2Freq());

  uint32_t pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4;
  uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8;
  uint32_t pllp = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 17;
  uint32_t pllq = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 21;
  uint32_t pllr = (RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25;

  my_printf("PLL raw: M=%lu, N=%lu, P=%lu, Q=%lu, R=%lu\n", 
            pllm, plln, pllp, pllq, pllr);

  my_printf("\nTimer test:\n");
  my_printf("TIM1 input: %lu Hz\n", HAL_RCC_GetPCLK1Freq());
  my_printf("TIM1 PSC: %lu\n", TIM1->PSC);
  my_printf("TIM1 ARR: %lu\n", TIM1->ARR);
  uint32_t tim1_freq = HAL_RCC_GetPCLK1Freq() / ((TIM1->PSC + 1) * (TIM1->ARR + 1));
  my_printf("TIM1 output: %lu Hz\n", tim1_freq);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifdef DEBUG
  my_printf("System ready - waiting for CAN data...\n");
#endif

  // ??нициализация структур для колес
  whl_chnl fl_whl_s = {numFL, &htim1, 0, 24, 0, 0, 0, 0, 0, 0};
  whl_chnl fr_whl_s = {numFR, &htim2, 0, 12, 0, 0, 0, 0, 0, 0};
  whl_chnl rl_whl_s = {numRL, &htim3, 0, 24, 0, 0, 0, 0, 0, 0};
  whl_chnl rr_whl_s = {numRR, &htim4, 0, 24, 0, 0, 0, 0, 0, 0};

  whl_arr[numFL] = &fl_whl_s;
  whl_arr[numFR] = &fr_whl_s;
  whl_arr[numRL] = &rl_whl_s;
  whl_arr[numRR] = &rr_whl_s;

  for(int i = 0; i < 4; i++) {
      whl_arr[i]->pending_update = 0;
      whl_arr[i]->target_psc = 0;
      whl_arr[i]->target_arr = 0;
  }

  set_new_speeds(0,0,0,0, whl_arr);

  HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);

#ifdef DEBUG
  test_rpm_calculation();
#endif







  /* USER CODE BEGIN WHILE */
  while (1) {


    diagnostic_tim4_status();
    diagnostic_mode_status();
    diagnostic_gpio_status();
    diagnostic_alternative_functions();
    check_timer_interrupts();

#ifdef DEBUG
    static uint32_t last_debug = 0;
    if(HAL_GetTick() - last_debug > 500) {
        last_debug = HAL_GetTick();
        
        if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
            my_printf("[DEBUG] Channel mask: 0x%02X\n", g_system_state.channel_mask);
            my_printf("[DEBUG] TIM1 CEN=%d, TIM2 CEN=%d, TIM3 CEN=%d, TIM4 CEN=%d\n",
                     (TIM1->CR1 & TIM_CR1_CEN) ? 1 : 0,
                     (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0,
                     (TIM3->CR1 & TIM_CR1_CEN) ? 1 : 0,
                     (TIM4->CR1 & TIM_CR1_CEN) ? 1 : 0);
            my_printf("[DEBUG] whl_arr speeds: %d, %d, %d, %d\n",
                     whl_arr[0]->prev_speed,
                     whl_arr[1]->prev_speed,
                     whl_arr[2]->prev_speed,
                     whl_arr[3]->prev_speed);
        }
    }
#endif

    process_can_in_main();

    
    if (freshCanMsg == 1) {
        freshCanMsg = 0;
        my_printf("[PROCESS_CAN] ID: 0x%03X, Mode: %s, DLC: %d\n",
                  can_rx_msg.id, 
                  get_mode_name(g_system_state.current_mode),
                  can_rx_msg.dlc);
        
        if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
            int vFLrpm = ((uint16_t)canRX[0] << 8) | canRX[1];
            int vFRrpm = ((uint16_t)canRX[2] << 8) | canRX[3];
            int vRLrpm = ((uint16_t)canRX[4] << 8) | canRX[5];
            int vRRrpm = ((uint16_t)canRX[6] << 8) | canRX[7];
            
            set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);
            
            g_system_state.rpm_mode_active = 1;
            
            can_active_receiving = 1;
            recievingcounger = 4;
            
            g_system_state.last_can_command_time = HAL_GetTick();
        }
    }
    
    update_system_indicators();
    
    if (ms100Flag > 0) {
        ms100Flag = 0;
        HAL_GPIO_TogglePin(GPIOB, Out_1_Pin);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_GPIO_WritePin(SOLID_RELAY_CONTROL_GPIO_Port, SOLID_RELAY_CONTROL_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pins : SOLID_RELAY_CONTROL_Pin Out_1_Pin Out_2_Pin tim4_out_Pin */
  GPIO_InitStruct.Pin = SOLID_RELAY_CONTROL_Pin|Out_1_Pin|Out_2_Pin|tim4_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Blue_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void CANFD1_Set_Filtes(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};
    
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    
    sFilterConfig.FilterID1 = 0x004 << 5;
    sFilterConfig.FilterID2 = 0x003 << 5;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, 
        FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
#ifdef DEBUG
    my_printf("CAN: Classic mode, AutoRetransmission=ENABLED\n");
#endif
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
        return;
    }

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
#ifdef DEBUG
        my_printf("[CAN ERR] Failed to get RX message\n");
#endif
        return;
    }

#ifdef DEBUG
    my_printf("[CAN RX] ID: 0x%04X, DLC: %d, Data: ", 
              rxHeader.Identifier, 
              rxHeader.DataLength >> 16);
    
    for(int i = 0; i < 8; i++) {
        my_printf("%02X ", data[i]);
    }
    my_printf("\n");
#endif

    can_rx_msg.id = rxHeader.Identifier;
    can_rx_msg.dlc = rxHeader.DataLength >> 16;
    memcpy(can_rx_msg.data, data, 8);

    can_rx_pending = 1;
}

void process_can_in_main(void)
{
    if (can_rx_pending) {
        can_rx_pending = 0;
        
        if (can_rx_msg.id == 0x003) {
            if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
                freshCanMsg = 1;
                memcpy(canRX, can_rx_msg.data, 8);
            }
        }
        else if (can_rx_msg.id == CAN_CMD_ID) {
            process_can_command(can_rx_msg.data);
        }
    }
    
    if (g_system_state.pending_hi_z) {
        g_system_state.pending_hi_z = 0;
        enter_hi_impedance_mode();
    }
    
    if (can_tx_status_pending) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
            can_tx_status_pending = 0;
            send_system_status();
        }
    }
    
    if (can_tx_error_pending) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
            can_tx_error_pending = 0;
            send_error_response(can_tx_error_code);
        }
    }
}

static void led_blink_pattern(uint32_t current_time, uint32_t interval_ms)
{
    if(current_time - g_system_state.led_last_toggle_time >= interval_ms) {
        g_system_state.led_state = !g_system_state.led_state;
        g_system_state.led_last_toggle_time = current_time;
        
        if(g_system_state.led_state) {
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
        }
    }
}

void update_system_indicators(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(current_time - last_update < 10) {
        return;
    }
    last_update = current_time;
    
    switch(g_system_state.current_mode) {
        case MODE_BOOT:
            if(!g_system_state.led_boot_flashed) {
                HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_RESET);
                g_system_state.led_boot_start_time = current_time;
                g_system_state.led_boot_flashed = 1;
            }
            
            if(current_time - g_system_state.led_boot_start_time >= 1500) {
                HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            }
            break;
            
        case MODE_RPM_DYNAMIC:
            if(!g_system_state.rpm_mode_active) {
                HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            }
            break;
            
        case MODE_FIXED_FREQUENCY:
            led_blink_pattern(current_time, 500);
            break;
            
        case MODE_HI_IMPEDANCE:
            led_blink_pattern(current_time, 2500);
            break;
            
        case MODE_DISABLED:
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            break;
            
        case MODE_ERROR:
            led_blink_pattern(current_time, 50);
            break;
            
        default:
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            break;
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
#ifdef DEBUG
  my_printf("Wrong parameters value: file %s on line %d\r\n", file, line);
#endif
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
