# Python GUI Adaptation Brief - wss_emu v3

## STATUS (ID 0x006) - Структура и расшифровка

### Формат статус-сообщения (8 байт):
```
Byte 0: Режим работы
  0x01 = MODE_RPM_DYNAMIC
  0x02 = MODE_FIXED_FREQUENCY
  0x10 = MODE_HI_IMPEDANCE
  0x00 = MODE_DISABLED

Byte 1: Channel mask (активные колёса)
  Бит 0 = FL (TIM1)
  Бит 1 = FR (TIM2)
  Бит 2 = RL (TIM3)
  Бит 3 = RR (TIM4)
  Пример: 0x0F = все 4 активны

Bytes 2-3: Частота × 10 (uint16_t LE)
  Только для MODE_FIXED_FREQUENCY!
  Пример: 0x2710 = 10000 (значит 1000.0 Hz)
  Диапазон: 0.1 Hz - 6553.5 Hz

Byte 4: Флаги состояния
  Бит 0: Analog signal (не используется, всегда 0)
  Бит 1: Hi-Z mode active (1 = да, 0 = нет)

Bytes 5-6: Uptime в секундах (uint16_t LE)
  Время работы системы с момента старта

Byte 7: Reserved (всегда 0)
```

---

## MODE_RPM_DYNAMIC (0x01)

### Команда входа:
```
CAN ID: 0x004
Data: 01 FF 00 00 00 00 00 00
  [0x01] = CMD_SET_RPM_MODE
  [0xFF] = All wheels
```

### Входные данные (постоянно, каждые ~1ms):
```
CAN ID: 0x003
Data: [FL_H] [FL_L] [FR_H] [FR_L] [RL_H] [RL_L] [RR_H] [RR_L]
  Каждое значение = uint16_t BIG-ENDIAN
  Примеры:
    00 00 = выключено (< 0x3F = 63)
    13 88 = 5000 RPM
    27 10 = 10000 RPM
```

### Статус в RPM режиме:
```
Rx (0x006): 01 0F 00 00 00 00 9C 00
  [01] = MODE_RPM_DYNAMIC
  [0F] = Channel mask (зависит от того какие RPM приходят)
  [00 00] = частота НЕ используется в RPM режиме!
  [00] = флаги (обычно 0)
  [9C 00] = uptime в секундах (156 сек)
```

⚠️ **ВАЖНО в RPM режиме:**
- Bytes 2-3 (частота) **ВСЕГДА = 0x0000** (не используется)
- Статус обновляется динамически в зависимости от приходящих RPM
- Channel mask отражает какие колёса активны (RPM > 0x3F)

---

## MODE_FIXED_FREQUENCY (0x02)

### Команда входа:
```
CAN ID: 0x004
Data: 02 [MASK] [Freq_LE_4bytes] 00 00
```

### Два подрежима:

#### ALL_FOUR (маска 0x0F)
```
Отправляемая команда:
  02 0F [freq_mhz_LE] 00 00
  
Пример: 999.935 Hz выходная частота
  (таймер = 1999.87 Hz с учётом toggle)
  02 0F FE 83 1E 00 00 00
  
Статус ответ:
  02 0F 84 27 00 00 9C 00
    [02] = MODE_FIXED_FREQUENCY
    [0F] = все 4 активны
    [84 27] = 10116 (частота × 10) = 1011.6 Hz
    [00] = флаги
    [9C 00] = uptime
```

#### ONLY_FR (маска 0x02)
```
Отправляемая команда:
  02 02 [freq_mhz_LE] 00 00
  
Пример: 999.935 Hz выходная частота
  (таймер = 1999.87 Hz с учётом toggle)
  02 02 FE 83 1E 00 00 00
  
Статус ответ:
  02 02 84 27 00 00 9C 00
    [02] = MODE_FIXED_FREQUENCY
    [02] = ТОЛЬКО FR (TIM2) активен
    [84 27] = 10116 (частота × 10) = 1011.6 Hz
    [00] = флаги
    [9C 00] = uptime
    
ПРИМЕЧАНИЕ: TIM1, TIM3, TIM4 ВЫКЛЮЧЕНЫ!
```

⚠️ **КРИТИЧНО:**
- ALL_FOUR использует 16-бит таймеры, точность ~0.01 Hz
- ONLY_FR использует 32-бит TIM2, точность ~0.001 Hz
- Все таймеры в TOGGLE режиме: выходная freq = таймер / 2
- Поэтому **отправляемая частота всегда в 2 раза больше желаемой**!

---

## MODE_HI_IMPEDANCE (0x10)

### Команда входа:
```
CAN ID: 0x004
Data: 06 00 00 00 00 00 00 00
  [0x06] = CMD_SET_HI_IMPEDANCE
```

### Статус в Hi-Z режиме:
```
Rx (0x006): 10 0F 00 00 02 00 9C 00
  [10] = MODE_HI_IMPEDANCE
  [0F] = Channel mask (не меняется)
  [00 00] = частота = 0
  [02] = Бит 1 SET (Hi-Z mode active = 1)
  [9C 00] = uptime
```

⚠️ **ВАЖНО в Hi-Z:**
- PB10 (SSR) установлен в HIGH
- PA15 (FR) переводится в INPUT (Hi-Z)
- Флаг byte[4] & 0x02 = Hi-Z active indicator
- Выход из Hi-Z: отправить любую другую команду (0x01, 0x02)

---

## Python GUI - Что нужно обновить

### 1. **Парсинг Status сообщения (ID 0x006)**
```python
def parse_status(data):
    """data = bytes из CAN сообщения (8 байт)"""
    mode = data[0]
    channel_mask = data[1]
    freq_x10 = struct.unpack('<H', data[2:4])[0]  # Little-endian!
    flags = data[4]
    uptime = struct.unpack('<H', data[5:7])[0]
    
    # Режимы
    mode_name = {
        0x01: "RPM_DYNAMIC",
        0x02: "FIXED_FREQUENCY",
        0x10: "HI_IMPEDANCE",
        0x00: "DISABLED"
    }[mode]
    
    # Флаги
    hi_z_active = (flags & 0x02) != 0
    
    # Частота (только для FIXED_FREQUENCY!)
    if mode == 0x02:
        freq_hz = freq_x10 / 10.0
    else:
        freq_hz = 0  # RPM и Hi-Z не используют это поле
    
    return {
        'mode': mode_name,
        'channels': channel_mask,
        'frequency_hz': freq_hz,
        'hi_z_active': hi_z_active,
        'uptime_sec': uptime
    }
```

### 2. **Отправка FIXED_FREQUENCY команд**

⚠️ **ПОМНИ: все таймеры в toggle режиме!**

```python
def send_fixed_all_four(bus, desired_output_hz):
    """desired_output_hz = желаемая частота на выходе"""
    timer_freq = desired_output_hz * 2  # Toggle поделит пополам!
    freq_mhz = int(timer_freq * 1000)
    
    data = bytearray(8)
    data[0] = 0x02
    data[1] = 0x0F  # ALL_FOUR
    data[2:6] = struct.pack('<I', freq_mhz)
    
    msg = can.Message(arbitration_id=0x004, data=data)
    bus.send(msg)

def send_fixed_only_fr(bus, desired_output_hz):
    """desired_output_hz = желаемая частота на выходе"""
    timer_freq = desired_output_hz * 2  # Toggle поделит пополам!
    freq_mhz = int(timer_freq * 1000)
    
    data = bytearray(8)
    data[0] = 0x02
    data[1] = 0x02  # ONLY_FR
    data[2:6] = struct.pack('<I', freq_mhz)
    
    msg = can.Message(arbitration_id=0x004, data=data)
    bus.send(msg)
```

### 3. **UI - Слайдеры для двух режимов**

```
[FIXED FREQUENCY TAB]

┌─────────────────────────────────────┐
│ Select Mode:                        │
│  ○ ALL_FOUR (все 4 колеса)         │
│  ○ ONLY_FR (максимум точности)     │
├─────────────────────────────────────┤
│ Frequency: [========●====] 999.935  │
│ Range: 0.1 - 4500 Hz                │
├─────────────────────────────────────┤
│ [SEND]  Status: ✓ ALL_FOUR active   │
│         Actual: 999.94 Hz           │
│         Error: 0.005 Hz (0.0005%)   │
└─────────────────────────────────────┘
```

### 4. **Status Display**

```
Mode: FIXED_FREQUENCY
├─ Wheel mask: 0x0F (FL FR RL RR)
├─ Target: 999.935 Hz
├─ Actual: 999.94 Hz
├─ Error: 0.0005%
├─ Hi-Z: INACTIVE
└─ Uptime: 156 sec
```

---

## Контрольный список

- [ ] Парсер status сообщения обновлён
- [ ] ALL_FOUR и ONLY_FR кнопки добавлены
- [ ] Частота умножается на 2 при отправке (toggle компенсация)
- [ ] Bytes 2-3 status игнорируются в RPM режиме
- [ ] Hi-Z индикатор показывает флаг byte[4] & 0x02
- [ ] Channel mask обновляется динамически в RPM
- [ ] Uptime отображается корректно (uint16_t LE)

---

## Примеры тестирования

### Test 1: RPM режим
```
1. Отправить 0x01 (RPM mode)
2. Отправлять RPM данные (CAN ID 0x003)
3. Статус должен показать MODE_RPM, freq = 0
4. Channel mask должен меняться в зависимости от RPM
```

### Test 2: FIXED ALL_FOUR
```
1. Отправить 02 0F FE 83 1E 00 00 00 (999.935 Hz)
2. Статус: режим FIXED_FREQUENCY, mask 0x0F, freq ~1000 Hz
3. Осциллограф: все 4 вывода ~999.935 Hz
```

### Test 3: FIXED ONLY_FR
```
1. Отправить 02 02 FE 83 1E 00 00 00 (999.935 Hz)
2. Статус: режим FIXED_FREQUENCY, mask 0x02, freq ~1000 Hz
3. Осциллограф: только PA15 работает, остальные OFF
```

### Test 4: Hi-Z режим
```
1. Отправить 06 00 00 00 00 00 00 00
2. Статус: режим HI_IMPEDANCE, флаг Hi-Z = active
3. Проверить PB10 = HIGH, PA15 = Hi-Z (INPUT)
```

---

**Версия:** 3.1  
**Дата:** Январь 2025  
**Статус:** Ready for implementation
