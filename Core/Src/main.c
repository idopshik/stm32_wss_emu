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

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;

uint8_t canRX[8];  // CAN Bus Receive Buffer
uint8_t freshCanMsg = 0;

char ms100Flag = 0;
char ms100Flag_2 = 0;

uint8_t recievingcounger = 0;    // for LED logic
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
    uint8_t initial_tmp_flag;  // Добавляем недостающий член
    uint8_t psc_change_flag;   // Добавляем недостающий член
} whl_chnl;

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

// Новые функции для работы с режимами
void update_system_indicators(void);
void handle_mode_led_indication(void);
void check_system_health(void);

void test_mode_switching(void);  // Для тестирования через UART

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
            printf("%#x ", data_arr[i]);
        }
    }
    printf("\n");
}

uint32_t calculete_period_only(int val)
{
    // For 32bit timer
    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;
}

int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum)
{
    float factor = 0.02;
    int newpresc;

    if (val < 4274) {
        newpresc = 1200;
    }
    else {
        newpresc = 24;
    }

    // Исправляем обращение к initial_tmp_flag
    if (whl_arr[wheelnum]->initial_tmp_flag == 0) {
        whl_arr[wheelnum]->initial_tmp_flag = 1;
        arr[0] = 24;
    }
    else {
        arr[0] = whl_arr[wheelnum]->prev_psc;
    }
    whl_arr[wheelnum]->prev_psc = newpresc;

    uint32_t tmp;
    tmp = (APB1_CLK / (val * factor * MinutTeethFactor * (arr[0] + 1))) - 1;
    if (tmp > 65536) {
        tmp = 65535;
    }
    arr[2] = (int)tmp;
    
    if (arr[0] != arr[1]) {
        tmp = ((APB1_CLK / (val * factor * MinutTeethFactor * (newpresc + 1))) - 1);
        if (tmp > 65536) {
            tmp = 65535;
        }
        arr[3] = (int)tmp;
    }
    else {
        arr[3] = arr[2];
    }

    return 0;
}

void update_wheel_seamless(TIM_TypeDef* TIMx, int rpm, whl_chnl* wheel) {
    if (rpm < 0x3F) {
        TIMx->CR1 &= ~TIM_CR1_CEN;
        wheel->pending_update = 0;
        __HAL_TIM_DISABLE_IT(wheel->htim, TIM_IT_UPDATE);
        return;
    }
    
    int calc[4];
    calculete_prsc_and_perio(rpm, calc, whl_arr, wheel->wheel_num);
    
    int new_psc = calc[1];
    int new_arr = calc[3];
    
    wheel->prev_arr = TIMx->ARR;
    
    if (new_psc == wheel->prev_psc) {
        TIMx->CR1 |= TIM_CR1_ARPE;
        TIMx->ARR = new_arr;
        wheel->prev_psc = new_psc;
        wheel->prev_speed = rpm;
        wheel->pending_update = 0;
        __HAL_TIM_DISABLE_IT(wheel->htim, TIM_IT_UPDATE);
        
        if (!(TIMx->CR1 & TIM_CR1_CEN)) {
            TIMx->CR1 |= TIM_CR1_CEN;
        }
        return;
    }
    
    wheel->target_psc = new_psc;
    wheel->target_arr = new_arr;
    wheel->pending_update = 1;
    
    __HAL_TIM_ENABLE_IT(wheel->htim, TIM_IT_UPDATE);
    
    if (!(TIMx->CR1 & TIM_CR1_CEN)) {
        TIMx->CR1 |= TIM_CR1_CEN;
    }
    
printf("PSC change pending: %d->%d, ARR: %lu->%d\n", 
       wheel->prev_psc, new_psc, wheel->prev_arr, new_arr);
}

void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[]) {
    if (vFLrpm != whl_arr[numFL]->prev_speed) {
        update_wheel_seamless(TIM1, vFLrpm, whl_arr[numFL]);
    }
    
    if (vFRrpm != whl_arr[numFR]->prev_speed) {
        if (vFRrpm < 0x3F) {
            TIM2->CR1 &= ~TIM_CR1_CEN;
        } else {
            TIM2->CR1 |= TIM_CR1_CEN;
            uint32_t period = calculete_period_only(vFRrpm);
            TIM2->CR1 |= TIM_CR1_ARPE;
            TIM2->ARR = period;
        }
        whl_arr[numFR]->prev_speed = vFRrpm;
    }
    
    if (vRLrpm != whl_arr[numRL]->prev_speed) {
        update_wheel_seamless(TIM3, vRLrpm, whl_arr[numRL]);
    }
    
    if (vRRrpm != whl_arr[numRR]->prev_speed) {
        update_wheel_seamless(TIM4, vRRrpm, whl_arr[numRR]);
    }
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

void print_timer_status(void) {
    if (ms100Flag_2) {
        ms100Flag_2 = 0;
        printf("TIM1: PSC=%lu, ARR=%lu, CNT=%lu%s\n", 
               TIM1->PSC, TIM1->ARR, TIM1->CNT,
               whl_arr[numFL]->pending_update ? " [PENDING]" : "");
        printf("TIM3: PSC=%lu, ARR=%lu, CNT=%lu%s\n", 
               TIM3->PSC, TIM3->ARR, TIM3->CNT,
               whl_arr[numRL]->pending_update ? " [PENDING]" : "");
        printf("TIM4: PSC=%lu, ARR=%lu, CNT=%lu%s\n", 
               TIM4->PSC, TIM4->ARR, TIM4->CNT,
               whl_arr[numRR]->pending_update ? " [PENDING]" : "");
    }
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
            whl_arr[i]->prev_speed = whl_arr[i]->prev_speed;
            whl_arr[i]->pending_update = 0;
            
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
            
            printf("PSC seamless switch: TIM%c, PSC=%d, ARR=%d\n", 
                   '1' + i, whl_arr[i]->target_psc, whl_arr[i]->target_arr);
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
        HAL_GPIO_TogglePin(GPIOB, tim4_out_Pin);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
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

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    printf("Seamless PSC switching enabled\n");

// Инициализация системы режимов
system_init_modes();

    // Инициализация системы режимов
    system_init_modes();

    // Переходим в режим RPM (стандартный режим работы)
    system_switch_mode(MODE_RPM_DYNAMIC);

    printf("\n========================================\n");
    printf("SYSTEM STARTED SUCCESSFULLY\n");
    printf("Current mode: %s\n", get_mode_name(g_system_state.current_mode));
    printf("Waiting for CAN commands...\n");
    printf("========================================\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int counter = 0;

    printf("System ready - waiting for CAN data...\n");

// Правильная инициализация структур
whl_chnl fl_whl_s = {numFL, &htim1, 0, 24, 0, 0, 0, 0, 0, 0};
whl_chnl fr_whl_s = {numFR, &htim2, 0, 12, 0, 0, 0, 0, 0, 0};
whl_chnl rl_whl_s = {numRL, &htim3, 0, 24, 0, 0, 0, 0, 0, 0};
whl_chnl rr_whl_s = {numRR, &htim4, 0, 24, 0, 0, 0, 0, 0, 0};

    whl_chnl *p_fl_whl = &fl_whl_s;
    whl_chnl *p_fr_whl = &fr_whl_s;
    whl_chnl *p_rl_whl = &rl_whl_s;
    whl_chnl *p_rr_whl = &rr_whl_s;

    whl_arr[numFL] = p_fl_whl;
    whl_arr[numFR] = p_fr_whl;
    whl_arr[numRL] = p_rl_whl;
    whl_arr[numRR] = p_rr_whl;

    for(int i = 0; i < 4; i++) {
        whl_arr[i]->pending_update = 0;
        whl_arr[i]->target_psc = 0;
        whl_arr[i]->target_arr = 0;
    }

    set_new_speeds(0,0,0,0, whl_arr);

    HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);

    /* USER CODE BEGIN WHILE */
    while (1) {
        // Существующая обработка CAN сообщений RPM
        if (freshCanMsg == 1) {
            freshCanMsg = 0;
            
            // Обрабатываем RPM данные ТОЛЬКО если в RPM режиме
            if(g_system_state.current_mode == MODE_RPM_DYNAMIC) {
                HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
                
                int vFLrpm, vFRrpm, vRLrpm, vRRrpm;
                vFLrpm = ((uint16_t)canRX[0] << 8) | canRX[1];
                vFRrpm = ((uint16_t)canRX[2] << 8) | canRX[3];
                vRLrpm = ((uint16_t)canRX[4] << 8) | canRX[5];
                vRRrpm = ((uint16_t)canRX[6] << 8) | canRX[7];
                
                printf("RPM: FL=%d, FR=%d, RL=%d, RR=%d\n", 
                       vFLrpm, vFRrpm, vRLrpm, vRRrpm);
                
                set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);
                
                can_active_receiving = 1;
                recievingcounger = 4;
            } else {
                // В других режимах игнорируем RPM сообщения
                printf("[INFO] Ignoring RPM data (not in RPM mode)\n");
            }
            
            // Обновляем время последней CAN команды
            g_system_state.last_can_command_time = HAL_GetTick();
        }
        
        // Обновление индикации (вызываем в основном цикле)
        update_system_indicators();
        
        // Проверка здоровья системы
        check_system_health();
        
        // Существующая обработка таймеров
        if (ms100Flag > 0) {
            ms100Flag = 0;
            HAL_GPIO_TogglePin(GPIOB, Out_1_Pin);
        }
        
        print_timer_status();
    }
    /* USER CODE END WHILE */
}

// Остальные функции инициализации (SystemClock_Config, MX_FDCAN1_Init, MX_TIMx_Init, MX_USART1_UART_Init, MX_GPIO_Init)
// остаются без изменений из вашего исходного кода




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

// ... остальные функции инициализации периферии (FDCAN, TIM, USART, GPIO) 
// должны быть взяты из вашего исходного кода без изменений

/* USER CODE BEGIN 4 */

// ============================================
// ОБНОВЛЕНИЕ ИНДИКАЦИИ СИСТЕМЫ
// ============================================

void update_system_indicators(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Обновляем не чаще чем каждые 10 мс
    if(current_time - last_update < 10) {
        return;
    }
    last_update = current_time;
    
    // Обновление LED индикации
    handle_mode_led_indication();
}

// ============================================
// LED ИНДИКАЦИЯ РЕЖИМОВ
// ============================================

void handle_mode_led_indication(void)
{
    uint32_t interval_ms = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Определяем интервал мигания в зависимости от режима
    switch(g_system_state.current_mode) {
        case MODE_BOOT:
            interval_ms = 100;  // 5 Гц (быстрое)
            break;
            
        case MODE_RPM_DYNAMIC:
            interval_ms = 250;  // 2 Гц (среднее)
            break;
            
        case MODE_FIXED_FREQUENCY:
            interval_ms = 500;  // 1 Гц (медленное)
            break;
            
        case MODE_PWM:
            interval_ms = 1000; // 0.5 Гц (очень медленное)
            break;
            
        case MODE_ANALOG_FOLLOW:
            // В аналоговом режиме частота зависит от наличия сигнала
            if(g_system_state.analog_signal_present) {
                interval_ms = 250;  // 2 Гц (сигнал есть)
            } else {
                interval_ms = 1000; // 0.5 Гц (сигнала нет)
            }
            break;
            
        case MODE_DISABLED:
            // LED постоянно выключен
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);
            return;
            
        case MODE_ERROR:
            interval_ms = 50;  // 10 Гц (очень быстрое)
            break;
            
        default:
            interval_ms = 1000;
            break;
    }
    
    // Если интервал = 0 (режим выключен), выходим
    if(interval_ms == 0) {
        return;
    }
    
    // Проверяем, нужно ли переключить LED
    if(current_time - g_system_state.led_last_toggle_time >= interval_ms) {
        g_system_state.led_state = !g_system_state.led_state;
        g_system_state.led_last_toggle_time = current_time;
        
        // Управляем физическим LED
        if(g_system_state.led_state) {
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_RESET); // ВКЛ
        } else {
            HAL_GPIO_WritePin(GPIOA, EXT_LED_Pin, GPIO_PIN_SET);   // ВЫКЛ
        }
    }
}

// ============================================
// ПРОВЕРКА ЗДОРОВЬЯ СИСТЕМЫ
// ============================================

void check_system_health(void)
{
    static uint32_t last_check = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Проверяем не чаще чем раз в секунду
    if(current_time - last_check < 1000) {
        return;
    }
    last_check = current_time;
    
    // Проверка "зависания" - если долго нет команд
    if(g_system_state.current_mode != MODE_RPM_DYNAMIC) {
        // В режимах, кроме RPM, требуются периодические команды
        uint32_t time_since_last_cmd = current_time - g_system_state.last_can_command_time;
        
        if(time_since_last_cmd > 10000) { // 10 секунд нет команд
            printf("[WARNING] No CAN commands for %lu seconds\n", time_since_last_cmd / 1000);
            
            // Если в фиксированном режиме или ШИМ - это нормально
            // Если в аналоговом режиме - возможно, пропал сигнал
            if(g_system_state.current_mode == MODE_ANALOG_FOLLOW) {
                g_system_state.analog_signal_present = 0;
                printf("[WARNING] Analog signal lost?\n");
            }
        }
    }
    
    // Раз в 30 секунд выводим статус (для отладки)
    static uint32_t status_counter = 0;
    if(++status_counter >= 30) {
        status_counter = 0;
        system_print_status();
    }
}

static void CANFD1_Set_Filtes(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;
    sFilterConfig.FilterID2 = 0x07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    else {
        my_printf("filterOK\n\r");
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            Error_Handler();
        }
        else {
            if ((RxHeader1.Identifier != CAN_SPECIAL_ID)) {
                my_printf("wrongID: %#x \n\r", RxHeader1.Identifier);
            }
            else {
                freshCanMsg = 1;
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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
