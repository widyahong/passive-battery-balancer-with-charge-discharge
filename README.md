# Passive Battery Balancer (2S–5S) with Charge/Discharge Protection

A passive balancer with a discharge cutoff (weak cell), a charge cutoff (over-voltage,
dead cell, temperature) and charger detection. Supports **Li-ion/LiPo**
and **LiFePO4**, selectable over serial. 3S is the recommended size (see *Limitations*).

## 1. How it works

| Topic | Behaviour |
|---|---|
| When to balance | **Not continuously.** Only when the highest cell is above `bal_start` (near full) **and** a cell is `bal_delta` mV above the lowest. Hysteresis: stops at `bal_delta/2` or when the cell falls 20 mV below `bal_start`. Optional: only while a charger is present (`chg_mode 1`, `bal_chg_only 1`). |
| How | Each cell has a 51 Ω bleed resistor switched by a MOSFET through a PC817 (~80 mA at 4.1 V Li-ion). At most `max_bleed` bleeders at once (heat limit). |
| Measuring | Bleeders OFF and the sense optos ON only for the sampling window (`settle` ms, default 300 ms of each 5 s cycle). The dividers are disconnected from the cells the rest of the time. |
| Weak cell | Lowest cell ≤ `uv_cut` for `uv_delay` ms → **discharge FET OFF**. Back ON only when the lowest cell ≥ `uv_resume` (i.e. after charging). The lowest cell decides because a single series cell cannot be disconnected alone. |
| Over-voltage | Highest cell ≥ `ov` → **charge FET OFF** + forced bleeding. Back ON at ≤ `ov_resume`. |
| Dead cell | Any cell < `chg_min` (default 2.0 V) → charge FET stays OFF (no blind charging of a dead cell). Set `chg_min 0` to disable. |
| Temperature | `temp_src 2` (NTC): bleeding stops above `temp_max`, charging blocked above `chg_tmax` / below `chg_tmin`, discharge blocked above `dsg_tmax`. `temp_src 1` (internal ESP32 sensor) is only a coarse board-overheat guard for bleeding. |
| Charger detect | Optional input pin (`chg_mode 1`, polarity `chg_level`). Used to keep the MCU awake while charging, wake it from deep sleep when a charger is plugged in, and optionally to allow balancing only while charging. Without it, the firmware works from cell voltages only. |
| While locked | ESP32 deep-sleeps (wakes every 30 s, or on charger plug-in with `chg_mode 1`) so it does not drain the weak cell further. It stays awake if any cell is above `bal_start` or a charger is present. |
| Wiring fault | Any cell < 1.0 V or > 4.7 V (open sense wire, wrong `cells`) → both FETs OFF, no balancing. |
| Fail-safe | All FET gates have pulldowns and the firmware starts with both FETs **OFF** until the first valid reading. |

## 2. Universal MCU requirements

For N series cells:

- **N ADC inputs** (12-bit preferred, ≥10-bit works).
- **N + 3 digital outputs**: N bleeders, discharge FET, charge FET, sense-opto line (one pin for all taps).
- Optional: 1 digital input (charger detect, ideally a wake-capable pin), 1 ADC (NTC), 1 LED, UART.
- Pins needed: **2S = 7, 3S = 9, 4S = 11, 5S = 13** (+3 optional).
- Divider values are for ≈2.4 V full scale (ESP32). For a 5 V MCU re-pick `R_TOP`/`R_BOT` in the sketch.
- Non-ESP32 boards compile too (generic `analogRead`, EEPROM for settings, no deep sleep, no internal temperature).

## 3. Bill of materials (all through-hole)

| Qty | Part | Value / Model | Note |
|---|---|---|---|
| 1 | MCU board | **ESP32 DevKit (ESP32-WROOM-32)** | |
| N | Optocoupler | **PC817** (DIP-4) | one per bleeder |
| N | Bleed MOSFET | **IRLZ44N** (TO-220) | |
| N | Bleed resistor | **51 Ω, 1 W** | ≈0.34 W each |
| N | Opto LED resistor | **330 Ω, ¼ W** | |
| N | Bleed gate pulldown | **100 kΩ, ¼ W** | |
| N | Divider bottom | **100 kΩ, 1 %, ¼ W** | |
| N | Divider top | **75 k, 270 k, 430 k, 620 k, 820 k** (tap 1…5), 1 %, ¼ W | |
| N | ADC filter cap | **100 nF** | ADC pin to GND |
| N | Sense opto | **PC817** (DIP-4), phototransistor in series with `R_TOP` | disconnects each divider while idle |
| N | Sense-opto LED resistor | **3.3 kΩ, ¼ W** | ≈0.6 mA each, all from one GPIO |
| 1 | Sense-line pulldown | **100 kΩ, ¼ W** | GPIO22 to GND |
| 2 | Discharge / charge FET | **IRLZ44N** (or IRL540N) + heatsink, **100 Ω** gate resistor, **100 kΩ** pulldown each | ≈≤10 A; see Limitations |
| 1 | 5 V supply for ESP32 | **LM2596 buck module** (set 5.0 V, ≥25 V input) | or USB / separate supply |
| 1 | Fuse + holder | **5–10 A** | on PACK+ |
| – | Connectors, perfboard | JST-XH or screw terminals | |
| opt | NTC | **NTC 10 kΩ B3950** + **10 kΩ** | `temp_src 2` |
| opt | Charger-detect | see 4.5: jack with detect switch, or **PC817 + 1N5819 + 2.2 kΩ**, plus **100 kΩ** pull resistor | |

(N = number of cells.)

## 4. Circuit

### 4.1 Pack and ports: pick A or B

The firmware just drives two gates (`PIN_DSG`, `PIN_CHG`), so both options use the same code.
All FET drivers are low-side so **ESP32 GND = pack−**.

**A – separate charge and load ports (non-bidirectional, simplest to understand)**

```
 CHARGER +  ───────────────┐
 LOAD +     ───[FUSE]──────┴──── PACK+ ──► cells ──► PACK−
                                                       │
 CHARGER −  ──── D ┤ Q_CHG ├ S ───────────────────────┤
 LOAD −     ──── D ┤ Q_DSG ├ S ───────────────────────┴── GND (ESP32, buck)
```

**B – one bidirectional port, back-to-back FETs (common source)**

```
 PORT +  ───[FUSE]────────────────────────── PACK+
 PORT −  ──── D ┤ Q_DSG ├ S ──── M ──── S ┤ Q_CHG ├ D ──── PACK− = GND
                GPIO23                            GPIO25
```

How B blocks: Q_DSG OFF blocks discharge (its body diode blocks current from the port into the pack),
Q_CHG OFF blocks charging, and each FET's body diode lets the other direction through, so a locked
pack can still be charged and an over-charged pack can still be discharged. The midpoint M stays within
about 0.7 V of GND, so gates driven from the ESP32 still work (Q_DSG sees ~2.6 V gate drive while the pack
is discharging with Q_CHG off: weak for IRLZ44N, check on the bench).

**Gate drive for Q_DSG / Q_CHG (identical):**

```
 ESP32 GPIO ─[100Ω]─┬─── G  IRLZ44N
                    │
                 [100kΩ]
                    │
                   GND
```

### 4.2 One bleeder per cell (identical for every cell k)

Vk+ is the cell's positive tap, Vk− its negative (= tap k−1, or pack− for cell 1).
The PC817 is powered **by the cell itself**, so the module works at any position in the stack.

```
 Vk+ ●─────────┬───────────────────┐
               │                   │ PC817 pin 4 (C)
          [51Ω 1W]            ┌────┴─────┐          ESP32 GPIO ─[330Ω]─ PC817 pin 1 (A)
               │              │  PC817   │                        GND ─ PC817 pin 2 (K)
               │ D            └────┬─────┘
             ┌─┴─┐ IRLZ44N         │ pin 3 (E)
       G ────┤   │◄────────────────┼───────┐
             └─┬─┘                 │       │
               │ S                 │    [100kΩ]
 Vk− ●─────────┴───────────────────┴───────┘
```

### 4.3 Voltage dividers with a sense opto per tap

The PC817 phototransistor sits between the tap and `R_TOP` (pin 4 collector to the tap, pin 3 emitter to `R_TOP`).
All opto LEDs are fed from one GPIO through their own 3.3 kΩ.

```
 TAPk ──► pin 4 (C) ┐ PC817
                    ├ phototransistor
         pin 3 (E) ─┘──[R_TOP]──┬────────► ESP32 ADC1 pin
                                ├──[100nF]── GND
                                └──[100kΩ]── GND

 ESP32 GPIO22 ──┬─[3.3kΩ]── pin 1 (A)   PC817 of tap 1,  pin 2 (K) ── GND
                ├─[3.3kΩ]── pin 1 (A)   PC817 of tap 2,  pin 2 (K) ── GND
                ├─ ... one 3.3k + one opto LED per tap
                └─[100kΩ]── GND         (keeps the LEDs off while the pin is floating, e.g. at boot)

 R_TOP:  tap1 75k   tap2 270k   tap3 430k   tap4 620k   tap5 820k
```

- GPIO22 is HIGH only while sampling (≈0.6 mA per LED, ≈3 mA for 5 taps).
- When off, each ADC pin is held at ≈0 V by its 100 kΩ resistor, so the ESP32 protection diodes never conduct and nothing is back-fed.
- When on, the phototransistor carries only ≈20 µA, so it is deeply saturated and its drop is tiny. Calibration absorbs it.
- Settling is set by `R_TOP‖R_BOT × 100 nF` ≈ 4–9 ms, so the default `settle` of 300 ms is generous; 100 ms works too.
- The firmware subtracts neighbouring taps (cell k = tap k − tap k−1).

### 4.4 Optional NTC

```
 3V3 ─[10kΩ]─┬─► GPIO33
             └─[NTC 10k]─ GND        (mount it against the bleed resistors / cells)
```

Internal ESP32 sensor (`temp_src 1`): no hardware. It reads the chip's own die (warmer than ambient, ±5 °C),
so it only guards the board; it is not used for charge/discharge limits.

### 4.5 Charger-detect input (GPIO27, optional)

Put a **100 kΩ pull resistor** on GPIO27 to its idle level (it also keeps the pin defined during deep sleep).
Set `chg_level` to the level that means "charger present".

| Source | Wiring | Setting |
|---|---|---|
| Jack with plug-detect switch | switch between 3V3 and GPIO27, 100 kΩ GPIO27 → GND | `chg_level 1` |
| Signal from a charger IC (PG / CHRG, open-drain) | GPIO27 to the pin, 100 kΩ pull-up to 3V3 | `chg_level 0` |
| Isolated, any charger (**option A only**) | port + → **1N5819** → **2.2 kΩ** → PC817 LED → port −; phototransistor: collector GPIO27 (100 kΩ pull-up to 3V3), emitter GND | `chg_level 0` |

The diode keeps the pack voltage from lighting the LED through Q_CHG when no charger is attached; it
also means the charge input must go through that diode (costs about 0.4 V × charge current). Option B
shares PACK+ between charger and load, so use the jack switch or a chip signal there.

## 5. ESP32 pinout (default in `firmware.ino`)

| Function | Cell 1 | Cell 2 | Cell 3 | Cell 4 | Cell 5 |
|---|---|---|---|---|---|
| Tap ADC (ADC1 only) | GPIO36 | GPIO39 | GPIO34 | GPIO35 | GPIO32 |
| Bleeder (→ 330 Ω → PC817) | GPIO16 | GPIO17 | GPIO18 | GPIO19 | GPIO21 |

Sense-opto LEDs **GPIO22** · Discharge FET **GPIO23** · Charge FET **GPIO25** · Charger detect **GPIO27** · NTC **GPIO33** · LED **GPIO2**.
Avoid strapping pins (0, 12, 15) for outputs.

## 6. Build, flash, calibrate

1. Arduino IDE → board **ESP32 Dev Module**; sketch in a folder named `firmware/`.
2. Power the ESP32 from USB **without** the pack, open the serial monitor at **115200**, send `help`.
3. Build dividers, sense optos and FET stage. Connect the pack: **pack−/GND first**, then B1+, B2+, B3+… ascending, then PACK+.
4. `set cells 3`, `profile liion` (or `lfp`), plus `set chg_mode 1` / `set temp_src 2` if wired, then `save`.
5. Calibrate with a multimeter (pack− to tap, in mV): `cal 1 4012`, `cal 2 8031`, `cal 3 12044`, then `save`.
6. Check each bleeder (`test 1 10`, ≈80 mA), and watch `dsg=` / `chg=` in the log. Test the cutoffs with a bench supply by temporarily raising `uv_cut` / lowering `ov`.
7. Keep the first charge supervised.

## 7. Serial commands (115200 baud)

| Command | Description |
|---|---|
| `status` | live line + all settings + calibration gains |
| `set <key> <value>` | change a setting |
| `profile liion` \| `profile lfp` | default thresholds for the chemistry |
| `cal <tap> <mV>` | calibrate a tap against a meter |
| `test <cell> <sec>` | force a bleeder ON (max 60 s, `0` cancels) |
| `log 0` \| `log 1` | periodic log off / on |
| `save` / `defaults` | store to flash / reset to defaults |

Log example: `C1=4012 C2=4031 C3=3998 | pack=12041 spread=33 | bleed=010 | dsg=1 chg=1 plug=0 T=31.2 | OK`

### Settings

| Key | Li-ion | LiFePO4 | Meaning |
|---|---|---|---|
| `cells` | 3 | 3 | series cells (2–5) |
| `bal_start` | 4050 | 3400 | balance only when a cell is above this (mV) |
| `bal_delta` | 30 | 30 | bleed cells this far above the lowest (mV) |
| `bal_chg_only` | 0 | 0 | 1 = balance only while charger present (`chg_mode 1`) |
| `ov` / `ov_resume` | 4300 / 4200 | 3700 / 3600 | charge FET OFF / back ON, forced bleed at `ov` (mV) |
| `chg_min` | 2000 | 2000 | no charging if a cell is below this (mV, 0 = off) |
| `uv_warn` | 3400 | 3100 | low warning (mV) |
| `uv_cut` / `uv_resume` | 3000 / 3300 | 2800 / 3000 | discharge FET OFF / allowed again (mV) |
| `uv_delay` | 3000 | 3000 | ms below `uv_cut` before cutting |
| `max_bleed` | 2 | 2 | simultaneous bleeders |
| `period` / `settle` | 5000 / 300 | 5000 / 300 | cycle time / sampling window (ms) |
| `temp_src` | 0 | 0 | 0 off, 1 internal, 2 NTC |
| `temp_max` | 60 | 60 | stop bleeding above (°C) |
| `chg_tmax` / `chg_tmin` | 45 / 0 | 45 / 0 | NTC only: charge limits (°C) |
| `dsg_tmax` | 60 | 60 | NTC only: discharge limit (°C) |
| `chg_mode` / `chg_level` | 0 / 1 | 0 / 1 | use charger-detect pin / level meaning "present" |

### LED

| Pattern | Meaning |
|---|---|
| short blip every 3 s | OK |
| 1 Hz blink | balancing |
| double blink | low-cell warning |
| 0.5 Hz slow | locked (low cell, discharge off) |
| ~3 Hz | over-voltage / dead cell / temperature |
| 5 Hz fast | wiring fault |

## 8. What drains the cells when idle

All figures below are estimates from the circuit and from my memory of the PC817 datasheet; measure your own build.

| Item | Effect |
|---|---|
| Bleeders (off) | ~nA (opto dark current). Negligible. |
| Sense optos (off) | Dark current only: on the order of 100 nA at most per tap, typically much less, flowing to GND through `R_BOT`. Worst case ≈0.5 µA for 5 taps. |
| Sampling (on, ≈6 % duty at the defaults) | The dividers load the cells below them. 3S at 4.0 V/cell: while on, cell 1 supplies ≈67 µA, cell 2 ≈44 µA, cell 3 ≈23 µA, so on average ≈4.0 / 2.7 / 1.4 µA. `settle 100` with `period 10000` gives ≈1 % duty, and the averages drop about 6×. |
| ESP32 + buck on the pack | Tens of mA while awake. It is series current, so every cell loses the same and the pack stays balanced. Deep sleep only happens in the locked state. |
| ESP32 on a separate supply | Nothing from the pack apart from the figures above. With the optos off (also when the ESP32 is unpowered) the dividers cannot back-feed it. |

The sampling load is still referenced to GND, so cell 1 gives a little more than cell 3 while a sample is running.
The average difference is a few µA, and the balancer evens it out while charging. Truly uniform sensing needs a
flying-capacitor stage or an AFE.

## 9. Limitations

- **Accuracy.** The ESP32 ADC is non-linear and each cell is the difference of two taps, so errors add up on higher taps. After calibration expect roughly ±15–30 mV per cell on 3S, worse on 4S/5S. Keep `bal_delta` ≥ 30 mV (40–50 on 5S). For ±2 mV, use an ADS1115 (Wire) or a dedicated AFE.
- **Internal temperature sensor** is coarse and measures the chip, not the pack.
- **Charge FET cannot fix a bad charger**: it only opens when a cell crosses `ov`; the charger still must be CC/CV for the pack voltage. The check runs every `period` (5 s by default), so a fast, small pack can overshoot in between; lower `period` for small packs.
- **Without a charger-detect pin**, a locked pack sleeps with the charge path kept open and wakes every 30 s; with a dead cell (`chg_min`), charging stays blocked and the pack needs preconditioning or `set chg_min 0` at your own risk.
- **Passive balancing is slow** (~80 mA) and works near full charge only.
- **Load FET**: IRLZ44N is only partly enhanced at 3.3 V gate drive. Fine for a few amps; for higher currents use a low-Rds(on) FET rated at 2.5–3 V Vgs or a gate driver.

## 10. Ideas if you take it further

Current sensing (INA219 + coulomb counting), ADS1115 or flying-capacitor measurement for cell-uniform sensing, WiFi/MQTT telemetry, an NTC on each bleeder.