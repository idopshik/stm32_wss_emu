#include "analog_follower.h"
#include "system_modes.h"

// Внешние переменные
extern system_state_t g_system_state;
extern void my_printf(const char *fmt, ...);

// Простая заглушка для аналогового режима
void analog_follower_init(void)
{
    my_printf("[ANALOG] Analog follower initialized\n");
    g_system_state.analog_signal_present = 0;  // Начинаем без сигнала
}

void analog_follower_process(void)
{
    static uint32_t last_toggle = 0;
    static uint8_t signal_state = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Симулируем наличие сигнала: 5 секунд есть, 5 секунд нет
    if(current_time - last_toggle > 5000) {
        last_toggle = current_time;
        signal_state = !signal_state;
        g_system_state.analog_signal_present = signal_state;
        
        if(signal_state) {
            my_printf("[ANALOG] Signal PRESENT (simulated)\n");
        } else {
            my_printf("[ANALOG] Signal ABSENT (simulated)\n");
        }
    }
}
