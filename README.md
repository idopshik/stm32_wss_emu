WSS EMULATOR v4 - ТЕХНИЧЕСКИЕ ТРЕБОВАНИЯ (АКТУАЛЬНО)

1. ОБЩИЕ ПРИНЦИПЫ

Назначение: Устройство эмулирует сигналы ABS-датчиков четырёх колёс на основе CAN-сообщений о скорости вращения.

Аппаратные особенности:

TIM1 (FL), TIM3 (RL), TIM4 (RR): 16-битные таймеры, переключение GPIO в прерывании

TIM2 (FR): 32-битный таймер, аппаратный toggle на PA15

Все выходы: Режим TOGGLE (выходная частота = частота таймера ÷ 2)

PA15 (FR): Основной канал для внешнего тестирования через SSR (PB10)

2. СТАТУСНОЕ СООБЩЕНИЕ (ID 0x006)

Формат (8 байт):

Байт	Назначение	Описание
0	Режим работы	0x01=RPM, 0x02=FIXED, 0x03=EXTERNAL
1	Channel mask	Битовая маска активных каналов (0x00-0x0F)
2-3	Частота ×10	Выходная_частота_Гц × 10 (только FIXED)
4	Флаги	Reserved (всегда 0x00)
5	Reserved	Всегда 0x00
6-7	Uptime (сек)	Время работы (uint16_t LE)

Особенности:

В RPM режиме байты 2-3 = 0x0000 (частота не рассчитывается)

В EXTERNAL режиме байты 2-3 = 0x0000 (частота не используется)

Примеры статуса:

text
RPM (активны FL и RL):    01 05 00 00 00 00 9C 00
FIXED 1000Hz ALL_FOUR:    02 0F 10 27 00 00 9C 00  (10000 = 1000.0 Hz)
FIXED 1000Hz ONLY_FR:     02 02 10 27 00 00 9C 00
EXTERNAL режим:           03 0F 00 00 00 00 9C 00

3. РЕЖИМ RPM_DYNAMIC (0x01)

3.1 Команда активации

text
CAN ID: 0x004
Data: 01 FF 00 00 00 00 00 00

3.2 Входные данные (CAN ID 0x003)

Формат: [FL_H FL_L FR_H FR_L RL_H RL_L RR_H RR_L]

16-битные значения Big-Endian

Коэффициент из .dbc: real_RPM = raw_value × 0.02

Порог активности: raw_value > 0x3F (63 десятичных)

3.3 ФИЗИЧЕСКАЯ ЛОГИКА ЭМУЛЯЦИИ

text
ABS датчик: 48 зубьев × 2 импульса/зуб = 96 импульсов/оборот

Частота сигнала НА ВЫХОДЕ GPIO (после деления на 2):
f_signal_Hz = real_RPM × 96 / 60 = real_RPM × 1.6

Объединённая формула: f_signal_Hz = raw_value × 0.02 × 1.6 = raw_value × 0.032

Частота ТАЙМЕРА (в 2 раза выше из-за TOGGLE):
f_timer_Hz = f_signal_Hz × 2 = raw_value × 0.064

3.4 АЛГОРИТМ РАСЧЁТА ТАЙМЕРОВ

Общий делитель:

text
Total_Divider = APB1_CLK / f_timer_Hz
              = 150,000,000 / (raw_value × 0.064)
              = 2,343,750,000 / raw_value  ← ПРАВИЛЬНАЯ КОНСТАНТА

1. TIM2 (FR, 32-битный):

text
PSC = 12 (ФИКСИРОВАННЫЙ в RPM режиме)
ARR = Total_Divider / (PSC + 1) - 1
    = 2,343,750,000 / (raw_value × 13) - 1

ВАЖНО: При возврате из FIXED в RPM режим PSC=12 восстанавливается явно
        в system_switch_mode(MODE_RPM_DYNAMIC) перед запуском таймера.

2. TIM1/TIM3/TIM4 (FL/RL/RR, 16-битные):

text
if (raw_value < 4274) {
    PSC = 1200
} else {
    PSC = 24
}
ARR = Total_Divider / (PSC + 1) - 1
if (ARR > 65535) ARR = 65535

3.5 ПРОВЕРОЧНЫЙ РАСЧЁТ

text
raw_value = 1000
f_signal = 1000 × 0.032 = 32 Гц (на выходе GPIO)
f_timer = 32 × 2 = 64 Гц (частота таймера)

Total_Divider = 2,343,750,000 / 1000 = 2,343,750

Для TIM2 (PSC=12):
ARR = 2,343,750 / 13 - 1 = 180,288 - 1 = 180,287
Проверка: f_timer = 150M / (13 × 180,288) ≈ 64 Гц ✓

Для TIM1 (PSC=24):
ARR = 2,343,750 / 25 - 1 = 93,750 - 1 = 93,749
Проверка: f_timer = 150M / (25 × 93,750) = 64 Гц ✓

4. РЕЖИМ FIXED_FREQUENCY (0x02)

Бескомпромиссный подход к точности:
- ONLY_FR (TIM2, 32-бит): точность 3 знака после запятой
- ALL_FOUR (16-бит):      точность 2 знака после запятой

Переключение частоты: SEAMLESS (без глитчей)
- Используется буферизация PSC и ARR через ARPE
- Оба регистра обновляются одновременно при естественном UPDATE event
- Никаких принудительных UG, никаких спинлоков
- Модуль: seamless_fixed.c / seamless_fixed.h

4.1 Команда установки частоты

text
CAN ID: 0x004
Data: 02 [MASK] [Freq_Timer_LE_4bytes] 00 00

MASK:
  0x0F = ALL_FOUR  (все 4 канала, 16-битная точность)
  0x02 = ONLY_FR   (только TIM2, 32-битная точность)

Freq_Timer: частота ТАЙМЕРА в миллигерцах (uint32_t LE)
           = Желаемая_выходная_частота_Гц × 2 × 1000

4.2 Примеры команд

text
Для 1000 Гц выходной:
  Частота таймера = 1000 × 2 = 2000 Гц
  В mHz = 2000 × 1000 = 2,000,000 mHz
  Hex: 0x001E8480
  Little-endian: 80 84 1E 00

Команда ALL_FOUR:  02 0F 80 84 1E 00 00 00
Команда ONLY_FR:   02 02 80 84 1E 00 00 00

4.3 Статус FIXED режима

c
// Корректная формула (исправлено):
freq_x10 = (uint16_t)(g_system_state.target_frequency_mhz / 200);
// target_frequency_mhz = 2,000,000 (частота таймера в mHz)
// /200 = 10,000 (0x2710) = 1000.0 Гц (выходная частота ×10) ✓

4.4 Seamless переключение частоты

Принцип:
- PSC всегда буферизован аппаратно (shadow register)
- ARR буферизован при ARPE=1
- Оба регистра применяются одновременно при UPDATE event

Алгоритм:
c
TIM->PSC       = new_psc;      // → preload, ждёт UPDATE
TIM->CR1      |= TIM_CR1_ARPE; // включить буферизацию ARR
TIM->ARR       = new_arr;      // → preload, ждёт UPDATE
// Текущий период доживает → UPDATE → оба значения применяются

Для TIM2 (hardware toggle): аппаратура продолжает переключать PA15 с новой частотой
Для TIM1/3/4 (interrupt):   UPDATE → прерывание → правильный toggle → новый период

Результат: один последний период на старой частоте, затем чистая новая частота.
           Нет глитчей, нет провалов, нет лишних импульсов.

4.5 Обработка channel_mask в system_modes.c

При входе в MODE_FIXED_FREQUENCY система проверяет маску:

c
if(g_system_state.channel_mask == 0x02) {
    // ONLY_FR: запустить только TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM2->CCER |= TIM_CCER_CC1E;
    // TIM1/3/4 остаются остановленными
} else if(g_system_state.channel_mask == 0x0F) {
    // ALL_FOUR: запустить все четыре таймера
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
    // + включить выходы CCER
}

Это предотвращает генерацию на неиспользуемых каналах при ONLY_FR.

5. РЕЖИМ EXTERNAL_SIGNAL (0x03)

5.1 Команда активации

text
CAN ID: 0x004
Data: 03 00 00 00 00 00 00 00

5.2 Действия устройства

Остановка всех таймеров (TIM1-4 CR1.CEN=0)

Перевод GPIO в INPUT mode (Hi-Z):

PA8 (FL), PA15 (FR), PA6 (RL), PB9 (RR) → INPUT

Активация SSR: PB10 = HIGH (питание оптопары для внешнего сигнала)

Установка режима: MODE_EXTERNAL_SIGNAL

5.3 Выход из EXTERNAL режима

Любая другая команда (RPM, FIXED) отключает EXTERNAL:

SSR: PB10 = LOW

GPIO → восстановление через restore_gpio_after_external():
  - PA15: GPIO_MODE_AF_OD (TIM2 hardware toggle)
  - PA8/PA6/PB9: GPIO_MODE_OUTPUT_PP (manual toggle в прерываниях)

6. РАСЧЁТ PSC/ARR ДЛЯ FIXED РЕЖИМА

6.1 Для 32-битного TIM2 (ONLY_FR)

Цель: максимальная точность (3 знака после запятой)

Алгоритм: перебор PSC от 0 до 65535, минимизация ошибки частоты

Для большинства частот PSC=0 даёт нулевую ошибку:
- output=1000 Hz → timer=2000 Hz → ARR=74999 (PSC=0, делитель 1)
- output=0.5 Hz  → timer=1 Hz    → ARR=149999999 (PSC=0)

Функция: calc_psc_arr_32bit() в seamless_fixed.c

6.2 Для 16-битных TIM1/3/4 (ALL_FOUR)

Ограничение: ARR ≤ 65535

Для низких частот требуется больший PSC:
- output=1 Hz → timer=2 Hz → target_ticks=75M → PSC=1143, ARR=65502

Функция: calc_psc_arr_16bit() в seamless_fixed.c

7. ФОРМУЛЫ ПРЕОБРАЗОВАНИЯ (СПРАВОЧНИК)

7.1 RPM → Частота

text
Выходная частота GPIO:   f_signal_Hz = raw_RPM × 0.032
Частота таймера:         f_timer_Hz = raw_RPM × 0.064
Общий делитель:          Total_Divider = 2,343,750,000 / raw_RPM

7.2 FIXED → Команда

text
Желаемая выходная → команда: freq_mhz = output_Hz × 2 × 1000
Статус → реальная выходная:  output_Hz = status_freq_x10 / 10

7.3 Регистры таймера

text
(PSC + 1) × (ARR + 1) = Total_Divider = APB1_CLK / f_timer_Hz

Для TIM2: APB1_CLK = 150 МГц
Для расчёта: APB1_CLK = 150,000,000

8. ИСТОРИЯ ИЗМЕНЕНИЙ

Версия 4.0 → 4.1: Исправлены константы в wheel_control.c
  - calculete_period_only(): константа 2,343,750,000 вместо 4,687,500,000
  - calculete_prsc_and_perio(): та же константа для 16-бит таймеров
  - Результат: выходная частота стала правильной (была в 2 раза ниже)

Версия 4.1 → 4.2: Исправлен статус FIXED режима в can_commands.c
  - send_system_status(): freq_x10 = target_frequency_mhz / 200 (было /100)
  - Результат: статус возвращает выходную частоту, а не частоту таймера

Версия 4.2 → 4.3: Добавлен seamless frequency switching
  - Новый модуль: seamless_fixed.c / seamless_fixed.h
  - Функции: seamless_update_tim2(), seamless_update_tim16()
  - Принцип: буферизация PSC и ARR через ARPE, без UG и спинлоков
  - Результат: переключение частоты без глитчей в FIXED режиме

Версия 4.3 → 4.4: Исправлена обработка режимов в system_modes.c
  - MODE_RPM_DYNAMIC: восстановление TIM2->PSC = 12 при входе
  - MODE_FIXED_FREQUENCY: учёт channel_mask (ONLY_FR vs ALL_FOUR)
  - Результат: корректная работа при переходах между режимами

Версия документа: 4.4
Статус: Актуально, все баги исправлены
Дата: Февраль 2025
