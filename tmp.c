// ============================================================================
// ПОЧЕМУ CALLBACK НЕ ВЫЗЫВАЕТСЯ - ТОЧНАЯ ДИАГНОСТИКА ДЛЯ ТВОЕГО КОДА
// ============================================================================

// Я посмотрел твой код. Вот ТОЧНЫЕ проверки для ТВОЕГО проекта:

// ============================================================================
// ПРОВЕРКА 1: ПОРЯДОК ИНИЦИАЛИЗАЦИИ В main() (КРИТИЧЕСКАЯ!)
// ============================================================================

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  // MX инициализация:
  MX_GPIO_Init();
  MX_FDCAN1_Init();      // ← FDCAN ИНИЦИАЛИЗАЦИЯ
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  
  // ← ДОБАВЬ ПРОВЕРКУ:
  my_printf("[MAIN] After MX_FDCAN1_Init, before can_handler_init\n");
  
  can_handler_init();     // ← ВЫЗЫВАЕТСЯ ПОСЛЕ MX_FDCAN1_Init()? ДА!
  
  // ← ДОБАВЬ ПРОВЕРКУ:
  my_printf("[MAIN] can_handler_init() completed\n");
  
  system_init_modes();
  wheel_control_init();
  
  // Запуск таймеров:
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  // ...
  
  // ← ДОБАВЬ ПРОВЕРКУ:
  my_printf("[MAIN] All timers started, entering main loop\n");
  
  while (1) {
    can_process_in_main();
    HAL_Delay(1);
  }
}

// ============================================================================
// ПРОВЕРКА 2: NVIC ВКЛЮЧЕН ДЛЯ FDCAN1 В CUBEIDE
// ============================================================================

// ДЕЙСТВИЕ:
// 1. Открой STM32CubeIDE
// 2. Открой .ioc файл проекта
// 3. Слева нажми "NVIC" (или Connectivity → NVIC)
// 4. Найди "FDCAN1 interrupt"
// 5. Убедись что checkbox ВКЛЮЧЕН (галочка есть)
// 6. Priority должна быть от 0 до 15
//
// ЕСЛИ ОТКЛЮЧЕНО → ВКЛЮЧИ И СОХРАНИ!

// ============================================================================
// ПРОВЕРКА 3: FDCAN КОНФИГУРАЦИЯ В CUBEIDE
// ============================================================================

// ДЕЙСТВИЕ:
// 1. В том же .ioc файле найди FDCAN
// 2. Убедись что:
//    ✓ Mode: Normal
//    ✓ Frame Format: Classic CAN (не FD)
//    ✓ Prescaler: 1
//    ✓ TimeSeg1/TimeSeg2: совпадают с вашими значениями
//
// СОХРАНИ И ПЕРЕСОЗДАЙ КОД (Project → Generate Code)

// ============================================================================
// ПРОВЕРКА 4: В can_handler.c ДОБАВЬ ДИАГНОСТИКУ КАЖДОГО ШАГА
// ============================================================================

void can_handler_init(void)
{
    my_printf("[CAN INIT] ========== START ==========\n");
    
    FDCAN_FilterTypeDef sFilterConfig;

    // Фильтр 0
    my_printf("[CAN INIT] Configuring filter 0 (ID 0x003)...\n");
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;  // 0x003
    sFilterConfig.FilterID2 = 0x07FF;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        my_printf("[CAN ERROR] Filter 0 config FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Filter 0 OK\n");
    
    // Фильтр 1
    my_printf("[CAN INIT] Configuring filter 1 (ID 0x004)...\n");
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = CAN_CMD_ID;  // 0x004
    sFilterConfig.FilterID2 = 0x07FF;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        my_printf("[CAN ERROR] Filter 1 config FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Filter 1 OK\n");

    // Глобальный фильтр
    my_printf("[CAN INIT] Setting global filter...\n");
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
        FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
        != HAL_OK) {
        my_printf("[CAN ERROR] Global filter FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Global filter OK\n");

    // Прерывание
    my_printf("[CAN INIT] Activating RX_FIFO0 notification...\n");
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        my_printf("[CAN ERROR] ActivateNotification FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] Notification activated\n");
    
    // Запуск
    my_printf("[CAN INIT] Starting FDCAN...\n");
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        my_printf("[CAN ERROR] FDCAN Start FAILED!\n");
        Error_Handler();
    }
    my_printf("[CAN INIT] FDCAN started\n");
    
    my_printf("[CAN INIT] ========== COMPLETE ==========\n");
}

// ============================================================================
// ПРОВЕРКА 5: CALLBACK ИМЕЕТ ДИАГНОСТИКУ
// ============================================================================

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static uint32_t rx_count = 0;
    my_printf("[CALLBACK] Message #%lu\n", ++rx_count);
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        my_printf("[CALLBACK] RxFifo0ITs check passed\n");
        
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            my_printf("[CALLBACK ERROR] GetRxMessage failed!\n");
            Error_Handler();
        }
        
        my_printf("[CALLBACK] Got message ID: 0x%03lX\n", RxHeader1.Identifier);
        
        if (RxHeader1.Identifier == CAN_SPECIAL_ID) {
            freshCanMsg = 1;
            my_printf("[CALLBACK] → Set freshCanMsg = 1\n");
        }
        else if (RxHeader1.Identifier == CAN_CMD_ID) {
            memcpy(canCmdData, canRX, 8);
            freshCanCmd = 1;
            my_printf("[CALLBACK] → Set freshCanCmd = 1 (cmd=0x%02X)\n", canCmdData[0]);
        }
        else {
            my_printf("[CALLBACK WARN] Unknown ID: 0x%03lX\n", RxHeader1.Identifier);
        }
    }
}

// ============================================================================
// РЕЗУЛЬТАТ ДИАГНОСТИКИ
// ============================================================================

/*
ЗАГРУЗИ КОД И СМОТРИ КОНСОЛЬ:

ЕСЛИ видишь:
  [MAIN] After MX_FDCAN1_Init, before can_handler_init
  [CAN INIT] ========== START ==========
  [CAN INIT] Configuring filter 0 (ID 0x003)...
  [CAN INIT] Filter 0 OK
  ...
  [CAN INIT] ========== COMPLETE ==========
  [MAIN] can_handler_init() completed
  [MAIN] All timers started, entering main loop

→ can_handler_init() работает! Переходи к следующему шагу.

ОТПРАВЬ CAN СООБЩЕНИЕ ID 0x004, Data: 01 FF 00 00 00 00 00 00

ЕСЛИ видишь:
  [CALLBACK] Message #1
  [CALLBACK] RxFifo0ITs check passed
  [CALLBACK] Got message ID: 0x004
  [CALLBACK] → Set freshCanCmd = 1 (cmd=0x01)

→ ВСЁ РАБОТАЕТ! Проблема дальше в обработке.

ЕСЛИ ВИДИШЬ:
  [MAIN] After MX_FDCAN1_Init, before can_handler_init
  (потом ничего или Error_Handler зависание)

→ can_handler_init() вызвала Error_Handler()
→ Какой вывод видишь перед зависанием? Скажи какая ошибка.

ЕСЛИ ВИДИШЬ:
  [MAIN] After MX_FDCAN1_Init, before can_handler_init
  [CAN INIT] ========== START ==========
  [CAN INIT] Configuring filter 0 (ID 0x003)...
  (потом зависание)

→ HAL_FDCAN_ConfigFilter() вернула ошибку
→ Проверь NVIC в CubeIDE что FDCAN1 interrupt ВКЛЮЧЕН!

ЕСЛИ ВИДИШЬ:
  [MAIN] All timers started, entering main loop
  (отправишь CAN сообщение, но НИ ОДНОГО [CALLBACK] сообщения)

→ Callback вообще не вызывается
→ Либо сообщения не приходят, либо NVIC отключен, либо фильтр блокирует всё
→ Проверь:
   1. NVIC включен в CubeIDE?
   2. CAN трансивер подключен?
   3. CAN шина терминирована (120 Ом)?
   4. Baudrate совпадает с генератором?

*/

// ============================================================================
