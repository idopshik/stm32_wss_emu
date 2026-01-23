#ifndef ANALOG_FOLLOWER_H
#define ANALOG_FOLLOWER_H

#include "main.h"
#include "system_modes.h"

// Функции аналогового режима
void analog_follower_init(void);
void analog_follower_process(void);
void check_analog_signal(void);

#endif // ANALOG_FOLLOWER_H
