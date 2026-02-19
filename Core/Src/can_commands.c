
/**
 * can_commands.c - VERSION 4.3 (FIXED STATUS FREQUENCY)
 * 
 * Команды:
 * 0x01 = RPM MODE (динамический)
 * 0x02 = FIXED MODE (статичный)
 * 0x03 = EXTERNAL MODE (GPIO HIGH-Z)
 * 
 * ИСПРАВЛЕНИЕ (v4.3):
 *   send_system_status() возвращал частоту ТАЙМЕРА вместо выходной частоты GPIO.
 *   target_frequency_mhz хранит частоту таймера в миллигерцах
 *   (= output_Hz × 2 × 1000).
 *   Статус байты 2-3 должны содержать output_Hz × 10:
 *     freq_x10 = target_frequency_mhz / 200
 *              = (output_Hz × 2 × 1000) / 200 = output_Hz × 10  ✓
 *
 *   Старый (неверный) код:
 *     freq_x10 = target_frequency_hz * 10   (= timer_Hz × 10, в 2 раза больше)
 */

/**
 * can_commands.c - VERSION 4.4 (SEAMLESS FIXED FREQUENCY)
 *
 * Изменения v4.4:
 *   - FIXED MODE использует seamless_update_tim2 / seamless_update_tim16
 *   - Никакого TIM->CNT = 0, никакого UG при смене частоты
 *   - Функции calc_psc_arr_* перенесены в seamless_fixed.c
 *   - Статус возвращает выходную частоту (target_frequency_mhz / 200)
 */

#include "can_commands.h"
#include "seamless_fixed.h"
#include "system_modes.h"
#include <string.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern void my_printf(const char *fmt, ...);

static void send_can_message(uint32_t id, uint8_t *data, uint8_t length);

/* ============================================================
 *  ОБРАБОТКА CAN КОМАНД
 * ============================================================ */

void process_can_command(uint8_t *data)
{
    if (data == NULL) {
        my_printf("[CAN ERROR] NULL data\n");
        return;
    }

    uint8_t command = data[0];
    my_printf("[CAN] Command: 0x%02X\n", command);

    switch (command) {

    /* ===== 0x01: RPM MODE ===== */
    case 0x01: {
        uint8_t channel_mask = data[1];
        my_printf("[CAN] RPM MODE, channels: 0x%02X\n", channel_mask);

        g_system_state.channel_mask = (channel_mask == 0xFF) ? 0x0F : channel_mask;
        system_switch_mode(MODE_RPM_DYNAMIC);
        can_tx_status_pending = 1;
        break;
    }

    /* ===== 0x02: FIXED MODE ===== */
    case 0x02: {
        uint8_t channel_mask = data[1];

        /*
         * Freq_Timer: частота ТАЙМЕРА в миллигерцах (uint32_t LE)
         *   = output_Hz × 2 × 1000
         *
         * Примеры:
         *   output=1000 Hz → freq_mhz=2 000 000
         *   output=500  Hz → freq_mhz=1 000 000
         *   output=100  Hz → freq_mhz=200 000
         */
        uint32_t freq_mhz = ((uint32_t)data[2])        |
                            ((uint32_t)data[3] <<  8)   |
                            ((uint32_t)data[4] << 16)   |
                            ((uint32_t)data[5] << 24);

        /* freq_timer_hz — частота ТАЙМЕРА, не выходная */
        float freq_timer_hz = (float)freq_mhz / 1000.0f;

        my_printf("[CAN] FIXED MODE: timer=%.3f Hz (out=%.3f Hz), ch=0x%02X\n",
                  freq_timer_hz, freq_timer_hz / 2.0f, channel_mask);

        /* Диапазон: 0.5 Hz output (1 Hz timer) ... 2250 Hz output (4500 Hz timer) */
        if (freq_mhz < 1000 || freq_mhz > 4500000) {
            my_printf("[CAN ERROR] Frequency out of range: %lu mHz\n", freq_mhz);
            can_tx_error_pending = 1;
            can_tx_error_code    = 0x02;
            break;
        }

        /* Сохраняем для статуса */
        g_system_state.target_frequency_mhz = freq_mhz;
        g_system_state.target_frequency_hz   = (uint32_t)freq_timer_hz;
        g_system_state.channel_mask = (channel_mask == 0xFF) ? 0x0F : channel_mask;

        if (channel_mask == 0x02) {
            /* ---- ONLY_FR: только TIM2 (32-бит, hardware toggle) ---- */
            uint16_t psc;
            uint32_t arr;
            calc_psc_arr_32bit(freq_timer_hz, &psc, &arr);

            /*
             * seamless_update_tim2 пишет PSC и ARR в preload оба.
             * Применятся при следующем естественном переполнении TIM2.
             * PA15 продолжит переключаться аппаратно без глитчей.
             */
            seamless_update_tim2(psc, arr);

        } else if (channel_mask == 0x0F) {
            /* ---- ALL_FOUR: все 4 таймера ---- */
            uint16_t psc_16, arr_16;
            calc_psc_arr_16bit(freq_timer_hz, &psc_16, &arr_16);

            /*
             * seamless_update_tim16 для каждого из трёх 16-битных таймеров.
             * Пишем одни и те же PSC/ARR — у всех трёх одинаковая частота.
             * UPDATE events могут произойти не синхронно (таймеры не связаны),
             * но каждый канал переключится чисто, без лишних toggles.
             */
            seamless_update_tim16(TIM1, "TIM1", psc_16, arr_16);
            seamless_update_tim16(TIM3, "TIM3", psc_16, arr_16);
            seamless_update_tim16(TIM4, "TIM4", psc_16, arr_16);

            /* TIM2 рассчитываем отдельно (32-бит даст лучшую точность) */
            uint16_t psc_32;
            uint32_t arr_32;
            calc_psc_arr_32bit(freq_timer_hz, &psc_32, &arr_32);
            seamless_update_tim2(psc_32, arr_32);
        }

        /* Переход в режим (запускает таймеры если они были остановлены) */
        system_switch_mode(MODE_FIXED_FREQUENCY);

        can_tx_status_pending = 1;
        break;
    }

    /* ===== 0x03: EXTERNAL (Hi-Z) MODE ===== */
    case 0x03: {
        my_printf("[CAN] EXTERNAL MODE\n");
        system_switch_mode(MODE_EXTERNAL_SIGNAL);
        can_tx_status_pending = 1;
        break;
    }

    /* ===== 0x07: STATUS REQUEST ===== */
    case 0x07: {
        my_printf("[CAN] STATUS REQUEST\n");
        can_tx_status_pending = 1;
        break;
    }

    default: {
        my_printf("[CAN ERROR] Unknown command: 0x%02X\n", command);
        can_tx_error_pending = 1;
        can_tx_error_code    = 0x01;
        break;
    }
    }

    g_system_state.last_can_command_time = HAL_GetTick();
}

/* ============================================================
 *  ОТПРАВКА СТАТУСА
 * ============================================================ */

void send_system_status(void)
{
    HAL_Delay(25);  /* 25 ms turnaround */
    my_printf("[CAN TX] Status\n");

    uint8_t d[8] = {0};

    d[0] = (uint8_t)g_system_state.current_mode;
    d[1] = g_system_state.channel_mask;

    /*
     * Байты 2-3: output_Hz × 10 (только FIXED режим)
     *
     * target_frequency_mhz = output_Hz × 2 × 1000
     *   ÷ 200 = output_Hz × 10  ✓
     *
     * Пример: output=1000 Hz → mhz=2 000 000 → /200 = 10 000 → "1000.0 Hz"
     */
    uint16_t freq_x10 = 0;
    if (g_system_state.current_mode == MODE_FIXED_FREQUENCY) {
        freq_x10 = (uint16_t)(g_system_state.target_frequency_mhz / 200);
    }
    d[2] = freq_x10 & 0xFF;
    d[3] = (freq_x10 >> 8) & 0xFF;

    d[4] = 0;  /* flags: reserved */
    d[5] = 0;  /* reserved */

    uint32_t uptime = system_get_uptime_seconds();
    if (uptime > 65535) uptime = 65535;
    d[6] = uptime & 0xFF;
    d[7] = (uptime >> 8) & 0xFF;

    send_can_message(0x006, d, 8);
}

/* ============================================================
 *  ОТПРАВКА ОШИБКИ
 * ============================================================ */

void send_error_response(uint8_t error_code)
{
    my_printf("[CAN TX ERROR] code=0x%02X\n", error_code);

    uint8_t d[8] = {0};
    d[0] = 0xFF;
    d[1] = error_code;

    send_can_message(CAN_STATUS_ID, d, 8);
}

/* ============================================================
 *  ОТПРАВКА CAN (низкий уровень)
 * ============================================================ */

static void send_can_message(uint32_t id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier            = id;
    h.IdType                = FDCAN_STANDARD_ID;
    h.TxFrameType           = FDCAN_DATA_FRAME;
    h.DataLength            = FDCAN_DLC_BYTES_8;
    h.ErrorStateIndicator   = FDCAN_ESI_PASSIVE;
    h.BitRateSwitch         = FDCAN_BRS_OFF;
    h.FDFormat              = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    h.MessageMarker         = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, data);
}
