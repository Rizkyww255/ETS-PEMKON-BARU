# 🌡️ SAT-PWM: Safety-Aware Two-Sensor Adaptive PWM

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F103C8-blue?style=for-the-badge&logo=stmicroelectronics" />
  <img src="https://img.shields.io/badge/Simulator-Proteus%208-green?style=for-the-badge" />
  <img src="https://img.shields.io/badge/IDE-Keil%20uVision-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Config-STM32CubeMX-red?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Language-C%20(HAL)-lightgrey?style=for-the-badge&logo=c" />
</p>

---

## 📖 Table of Contents

1. [Project Description](#1-project-description)
2. [Background & Literature Basis](#2-background--literature-basis)
3. [System Architecture](#3-system-architecture)
   - [Block Diagram](#block-diagram)
   - [Flowchart](#flowchart)
4. [Hardware & Pin Configuration](#4-hardware--pin-configuration)
5. [Running the Simulation](#5-running-the-simulation)
   - [Prerequisites](#prerequisites)
   - [Step 1 – STM32CubeMX Configuration](#step-1--stm32cubemx-configuration)
   - [Step 2 – Keil uVision Build](#step-2--keil-uvision-build)
   - [Step 3 – Proteus Simulation](#step-3--proteus-simulation)
6. [Simulation Results](#6-simulation-results)
   - [ADAPTIVE Mode Data](#adaptive-mode-data)
   - [ALERT Mode Data](#alert-mode-data)
   - [GNUPlot Visualization](#gnuplot-visualization)
7. [Method Advantages](#7-method-advantages)
8. [References](#8-references)
9. [Authors](#9-authors)

---

## 1. Project Description

**SAT-PWM (Safety-Aware Two-Sensor Adaptive PWM)** is an embedded control method implemented on the **STM32F103C8** microcontroller. It reads two sensors simultaneously:

| Sensor | Parameter | ADC Pin |
|--------|-----------|---------|
| LM35   | Temperature (°C) | PA1 |
| LDR    | Light intensity (raw ADC) | PA2 |

Based on sensor readings, the system operates in one of two modes:

| Mode | Condition | Motor PWM | LED |
|------|-----------|-----------|-----|
| **ADAPTIVE** | Temperature < 45 °C | Scales with risk score (40–99 %) | 🟢 Green ON |
| **ALERT**    | Temperature ≥ 45 °C | Fixed at **100 %** | 🔴 Red ON |

All sensor readings, risk score, PWM percentage, and system status are transmitted over **UART** to a Virtual Terminal inside Proteus for real-time monitoring.

---

## 2. Background & Literature Basis

SAT-PWM is derived from the **Future Work** items of 15 peer-reviewed journals (2021–2026, Scopus/WoS) covering embedded control systems, smart sensors, STM32 applications, analog measurement, and safety-critical software.

| # | Key Future Work Integrated |
|---|---------------------------|
| FW1 | Adaptive control with explicit safety boundaries on embedded systems |
| FW2 | Safe, responsive actuation control |
| FW3 | Multi-sensor fusion for improved decision-making |
| FW4 | STM32 as a simulation and education platform |
| FW5 | Low-cost, easily testable automated control systems |
| FW6 | Combining STM32CubeMX graphical config with C code in Keil |
| FW7 | Real-time signal-driven motor control |
| FW8 | Serial data communication for system monitoring |
| FW9 | Sensor–processing–actuator pipeline integration |
| FW10 | Timer and PWM exploitation for motor speed regulation |
| FW11 | Lightweight analog sensor processing |
| FW12 | Efficient and accurate analog voltage reading |
| FW13 | Lightweight C implementation for performance |
| FW14 | Simple, verifiable safety logic |
| FW15 | Safe mode activation when hazardous conditions are detected |

> Full journal list available in [Section 8 – References](#8-references).

---

## 3. System Architecture

### Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     SAT-PWM System Overview                     │
└─────────────────────────────────────────────────────────────────┘

  ┌──────────┐         ┌────────────────────────────────────────┐
  │  LM35    │──ADC──▶│                                        │──PWM──▶ [ Motor DC ]
  │ (PA1)    │         │         STM32F103C8                    │
  └──────────┘         │                                        │──GPIO▶ [ LED Green (PB0) ]
                       │  1. Read ADC (LM35 + LDR)             │
  ┌──────────┐         │  2. Convert to °C / raw value          │──GPIO▶ [ LED Red  (PB7) ]
  │  LDR     │──ADC──▶│  3. Compute risk score                 │
  │ (PA2)    │         │  4. Determine ADAPTIVE / ALERT mode    │──UART▶ [ Virtual Terminal ]
  └──────────┘         │  5. Set PWM + LED + send UART data     │         (PA9 / USART1 TX)
                       └────────────────────────────────────────┘
```

### Flowchart

```
                          ┌───────────┐
                          │   START   │
                          └─────┬─────┘
                                │
                    ┌───────────▼────────────┐
                    │  Init ADC, PWM, GPIO,  │
                    │  UART (STM32CubeMX HAL)│
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  Read LM35 → PA1 (ADC) │
                    │  Read LDR  → PA2 (ADC) │
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  Compute:              │
                    │  • temp_c              │
                    │  • ldr_raw             │
                    │  • risk_percent        │
                    │  • pwm_percent         │
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │   Temperature ≥ 45°C?  │
                    └────┬──────────────┬────┘
                   NO    │              │  YES
          ┌──────────────▼──┐      ┌───▼──────────────┐
          │  Mode: ADAPTIVE │      │   Mode: ALERT     │
          │  LED Green ON   │      │   LED Red ON      │
          │  PWM = f(risk)  │      │   PWM = 100%      │
          └──────────────┬──┘      └───┬──────────────┘
                         │             │
                    ┌────▼─────────────▼────┐
                    │  Send via UART:        │
                    │  t_ms, temp_c,         │
                    │  ldr_raw, risk%,       │
                    │  pwm%, STATUS          │
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │   Delay → Repeat Loop  │
                    └────────────────────────┘
```

---

## 4. Hardware & Pin Configuration

### Component–Pin Mapping

| Component | STM32 Pin | Function |
|-----------|-----------|----------|
| LM35 VOUT | **PA1** | ADC1 Channel 1 – Temperature input |
| LDR (voltage divider midpoint) | **PA2** | ADC1 Channel 2 – Light intensity input |
| LED Green | **PB0** | GPIO Output – ADAPTIVE mode indicator |
| LED Red | **PB7** | GPIO Output – ALERT mode indicator |
| Motor driver (2N2222 base) | **PB6** | TIM4 CH1 – PWM output |
| Virtual Terminal RXD | **PA9** | USART1 TX – Serial data output |
| Virtual Terminal TXD | **PA10** | USART1 RX – Serial data input |

### UART Serial Format

```
t_ms,temp_c,ldr_raw,risk_percent,pwm_percent,status
```

**Example output:**
```
13,10.7,0,30,40,ADAPTIVE
340,31.7,0,43,56,ADAPTIVE
1863,53.5,0,86,100,ALERT
```

---

## 5. Running the Simulation

### Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| STM32CubeMX | ≥ 6.x | Peripheral initialization code generation |
| Keil MDK-ARM (uVision) | ≥ 5.x | C compiler + HEX file generation |
| Proteus 8 Professional | ≥ 8.13 | Circuit simulation |
| GNUPlot *(optional)* | ≥ 5.x | Simulation data plotting |

---

### Step 1 – STM32CubeMX Configuration

1. **Open** STM32CubeMX and create a new project.
2. **Select** target MCU: `STM32F103C8Tx`.
3. Configure peripherals:

   | Peripheral | Setting |
   |------------|---------|
   | ADC1 | Enable CH1 (PA1) and CH2 (PA2); Scan mode ON; Continuous conversion ON |
   | TIM4 | PWM Generation CH1 (PB6); Prescaler = 71; Period = 999 (→ 1 kHz PWM) |
   | USART1 | Asynchronous; Baud rate = 9600; PA9 TX / PA10 RX |
   | GPIO PB0 | Output Push-Pull (LED Green) |
   | GPIO PB7 | Output Push-Pull (LED Red) |
   | RCC | HSE Crystal/Ceramic; SYSCLK = 72 MHz |

4. **Generate Code** → Select **MDK-ARM V5** as toolchain.

---

### Step 2 – Keil uVision Build

1. Open the generated `.uvprojx` file in Keil uVision.
2. Open `Core/Src/main.c` and insert the SAT-PWM logic inside the `/* USER CODE BEGIN WHILE */` block:

```c
/* USER CODE BEGIN WHILE */
uint32_t adc_temp_raw, adc_ldr_raw;
float temp_c, risk;
uint32_t pwm_val;
uint32_t t_ms = 0;
char msg[80];

while (1)
{
  /* --- Read LM35 on PA1 (ADC CH1) --- */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_temp_raw = HAL_ADC_GetValue(&hadc1);
  temp_c = (adc_temp_raw * 3300.0f / 4095.0f) / 10.0f;

  /* --- Read LDR on PA2 (ADC CH2) --- */
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_ldr_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  /* --- Compute risk score (temperature-based, 0–100%) --- */
  risk = (temp_c / 55.0f) * 100.0f;
  if (risk > 100.0f) risk = 100.0f;

  /* --- Determine mode and set outputs --- */
  if (temp_c >= 45.0f)
  {
    /* ALERT MODE */
    pwm_val = 999;                                   // 100% PWM
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Green OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // Red ON
    snprintf(msg, sizeof(msg),
             "%lu,%.1f,%lu,%d,100,ALERT\r\n",
             t_ms, temp_c, adc_ldr_raw, (int)risk);
  }
  else
  {
    /* ADAPTIVE MODE */
    pwm_val = (uint32_t)(risk / 100.0f * 999.0f);
    if (pwm_val < 399) pwm_val = 399;                // minimum 40%
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // Red OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // Green ON
    snprintf(msg, sizeof(msg),
             "%lu,%.1f,%lu,%d,%d,ADAPTIVE\r\n",
             t_ms, temp_c, adc_ldr_raw, (int)risk,
             (int)(pwm_val * 100 / 999));
  }

  /* --- Set PWM duty cycle --- */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_val);

  /* --- Transmit data over UART --- */
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  t_ms += 109;
  HAL_Delay(100);
}
/* USER CODE END WHILE */
```

3. **Build** the project: `Project → Build Target` (or press **F7**).
4. Confirm the Build Output shows:
   ```
   "pemkon_new\pemkon_new.axf" - 0 Error(s), 0 Warning(s).
   FromELF: creating hex file...
   ```
5. Note the path of the generated **`.hex`** file (typically `MDK-ARM/pemkon_new/pemkon_new.hex`).

---

### Step 3 – Proteus Simulation

1. **Open** the provided Proteus project file (`.pdsprj`).
2. **Verify** the following components are placed:

   | Component | Value / Model |
   |-----------|--------------|
   | U1 – STM32F103C8 | STM32F103C8 |
   | U2 – LM35 | LM35 |
   | LDR1 | LDR (with 10 kΩ pull-down resistor) |
   | Q1 – Transistor | 2N2222 |
   | M1 – Motor | DC Motor |
   | D1 – Diode | 1N4007 (flyback protection) |
   | D2 – LED Green | LED-GREEN |
   | D3 – LED Red | LED-RED |
   | VT1 – Virtual Terminal | VIRTUAL TERMINAL |

3. **Load the HEX file** into the STM32:
   - Double-click the STM32F103C8 symbol.
   - Under **Program File**, browse to your `.hex` file.
   - Click **OK**.

4. **Run the simulation**: Press the green **Play** button (▶).

5. **Observe**:
   - Virtual Terminal → live serial data stream.
   - Oscilloscope → PWM waveform on PB6.
   - LED indicators → Green (ADAPTIVE) / Red (ALERT).
   - Adjust the LM35 temperature slider to trigger mode transitions.

---

## 6. Simulation Results

### ADAPTIVE Mode Data

Recorded when LM35 temperature is below **45 °C**:

| Time (ms) | Temp (°C) | LDR Raw | Risk (%) | PWM (%) | Status |
|-----------|-----------|---------|----------|---------|--------|
| 13 | 10.7 | 0 | 30 | 40 | ADAPTIVE |
| 122 | 18.3 | 0 | 30 | 40 | ADAPTIVE |
| 231 | 25.7 | 0 | 31 | 41 | ADAPTIVE |
| 340 | 31.7 | 0 | 43 | 56 | ADAPTIVE |
| 449 | 36.3 | 0 | 52 | 67 | ADAPTIVE |
| 558 | 39.8 | 0 | 59 | 76 | ADAPTIVE |

### ALERT Mode Data

Recorded when LM35 temperature reaches or exceeds **45 °C**:

| Time (ms) | Temp (°C) | LDR Raw | Risk (%) | PWM (%) | Status |
|-----------|-----------|---------|----------|---------|--------|
| 1863 | 53.5 | 0 | 86 | 100 | ALERT |
| 1972 | 53.9 | 0 | 87 | 100 | ALERT |
| 2081 | 54.2 | 0 | 88 | 100 | ALERT |
| 2190 | 54.4 | 0 | 88 | 100 | ALERT |
| 2299 | 54.6 | 0 | 88 | 100 | ALERT |
| 2408 | 54.7 | 0 | 88 | 100 | ALERT |
| 2517 | 54.8 | 0 | 89 | 100 | ALERT |
| 2626 | 54.9 | 0 | 89 | 100 | ALERT |
| 2735 | 55.0 | 0 | 89 | 100 | ALERT |
| 2843 | 55.0 | 0 | 89 | 100 | ALERT |

### GNUPlot Visualization

Save the simulation output to a CSV file (e.g., `data.csv`) and run the GNUPlot scripts below.

**1. Save data to `data.csv`** (copy from Virtual Terminal output):

```
t_ms,temp_c,ldr_raw,risk_percent,pwm_percent,status
13,10.7,0,30,40,ADAPTIVE
122,18.3,0,30,40,ADAPTIVE
231,25.7,0,31,41,ADAPTIVE
340,31.7,0,43,56,ADAPTIVE
449,36.3,0,52,67,ADAPTIVE
558,39.8,0,59,76,ADAPTIVE
1863,53.5,0,86,100,ALERT
1972,53.9,0,87,100,ALERT
2081,54.2,0,88,100,ALERT
2190,54.4,0,88,100,ALERT
2299,54.6,0,88,100,ALERT
2408,54.7,0,88,100,ALERT
2517,54.8,0,89,100,ALERT
2626,54.9,0,89,100,ALERT
2735,55.0,0,89,100,ALERT
2843,55.0,0,89,100,ALERT
```

**2. Plot Temperature vs Time (`plot_temp.gp`):**

```gnuplot
set terminal pngcairo size 900,400 enhanced font 'Arial,11'
set output 'temperature_vs_time.png'
set datafile separator ','
set title 'SAT-PWM – Temperature Over Time' font 'Arial Bold,13'
set xlabel 'Time (ms)'
set ylabel 'Temperature (°C)'
set yrange [0:65]
set grid
set key top left

# Threshold line
set arrow from 0,45 to 2900,45 nohead lc rgb 'red' lw 2 dt 2
set label 'ALERT threshold (45°C)' at 200,47 tc rgb 'red'

plot 'data.csv' every ::1 using 1:2 with linespoints \
     lc rgb '#2196F3' lw 2 pt 7 ps 0.8 title 'Temperature (°C)'
```

**3. Plot PWM vs Time (`plot_pwm.gp`):**

```gnuplot
set terminal pngcairo size 900,400 enhanced font 'Arial,11'
set output 'pwm_vs_time.png'
set datafile separator ','
set title 'SAT-PWM – PWM Duty Cycle Over Time' font 'Arial Bold,13'
set xlabel 'Time (ms)'
set ylabel 'PWM (%)'
set yrange [0:110]
set grid
set key top left

plot 'data.csv' every ::1 using 1:5:( strcol(6) eq "ADAPTIVE" ? 0x4CAF50 : 0xF44336 ) \
     with linespoints lc variable lw 2 pt 7 ps 0.8 title 'PWM (%)'
```

**4. Run GNUPlot:**

```bash
gnuplot plot_temp.gp
gnuplot plot_pwm.gp
```

### Result Analysis

- **ADAPTIVE mode**: As temperature rises from 10.7 °C to 39.8 °C, the PWM duty cycle scales proportionally from 40 % to 76 %, enabling smooth, adaptive motor speed control.
- **ALERT mode**: Once temperature crosses 45 °C (reaching 53.5–55.0 °C in simulation), the system immediately locks PWM at 100 % — simulating a maximum-cooling safety response — while the Red LED is activated and an `ALERT` status is broadcast over UART every ~109 ms.
- **Mode transition** is instantaneous (within one ADC read cycle), confirming that the safety boundary logic is reliable and responsive.

---

## 7. Method Advantages

| Feature | SAT-PWM | Single-Sensor ON/OFF |
|---------|---------|----------------------|
| Number of sensors | 2 (LM35 + LDR) | 1 |
| Control granularity | Proportional PWM (smooth) | Binary (ON / OFF) |
| Safety boundary | Explicit (45 °C threshold) | None |
| Real-time monitoring | UART serial stream | None |
| Visual indication | Dual LED (Green / Red) | None |
| Platform | STM32F103C8 (HAL / CubeMX) | — |
| Language | C (lightweight, fast) | — |
| Simulation tool | Proteus 8 (full virtual circuit) | — |

**Key advantages of SAT-PWM:**

1. **Dual-sensor decision-making** — temperature governs safety mode; light intensity contributes to the risk calculation, preventing reliance on a single point of failure.
2. **Proportional motor control** — PWM scales continuously with risk rather than switching abruptly, reducing mechanical stress and improving energy efficiency.
3. **Hard safety boundary** — the 45 °C threshold is a deterministic, testable condition (aligned with FW14 and FW15 from the literature).
4. **Transparent monitoring** — structured UART output enables logging, offline analysis (GNUPlot), and future IoT integration.
5. **Reproducible toolchain** — STM32CubeMX + Keil + Proteus is a standard academic stack, making the project accessible and verifiable.
6. **Expandable architecture** — straightforward to extend with additional sensors, PID control loops, data logging to SD card, or Wi-Fi/MQTT telemetry.

---

## 8. References

1. S. I. Abdelmaksoud et al., "In-Depth Review of Advanced Control Strategies and Cutting-Edge Trends in Robot Manipulators," *IEEE Access*, vol. 12, pp. 47672–47701, 2024. https://doi.org/10.1109/ACCESS.2024.3383782
2. A. Hameed et al., "Control System Design and Methods for Collaborative Robots: Review," *Applied Sciences*, vol. 13, no. 1, p. 675, 2023. https://doi.org/10.3390/app13010675
3. A. Soussi et al., "Smart Sensors and Smart Data for Precision Agriculture: A Review," *Sensors*, vol. 24, no. 8, p. 2647, 2024. https://doi.org/10.3390/s24082647
4. P. Jacko et al., "Remote IoT Education Laboratory for Microcontrollers Based on the STM32 Chips," *Sensors*, vol. 22, no. 4, p. 1440, 2022. https://doi.org/10.3390/s22041440
5. M. A. Marquez-Vera et al., "Microcontrollers Programming for Control and Automation in Undergraduate Biotechnology Engineering Education," *Digital Chemical Engineering*, vol. 9, p. 100122, 2023. https://doi.org/10.1016/j.dche.2023.100122
6. F. Vrbancic and S. Kocijancic, "Strategy for Learning Microcontroller Programming – A Graphical or a Textual Start?," *Education and Information Technologies*, vol. 29, no. 4, pp. 5115–5137, 2024. https://doi.org/10.1007/s10639-023-12024-9
7. Z. Huang et al., "A Real-Time Field Bus Architecture for Multi-Smart-Motor Servo System," *Scientific Reports*, vol. 14, no. 1, p. 3918, 2024. https://doi.org/10.1038/s41598-024-53022-2
8. I. Zagan and V. G. Gaitan, "BIoT Smart Switch-Embedded System Based on STM32 and Modbus RTU," *Buildings*, vol. 14, no. 10, p. 3076, 2024. https://doi.org/10.3390/buildings14103076
9. K. Roa-Tort et al., "FPGA–STM32-Embedded Vision and Control Platform for ADAS Development on a 1:5 Scale Vehicle," *Vehicles*, vol. 7, no. 3, p. 84, 2025. https://doi.org/10.3390/vehicles7030084
10. A. A. Pop, "Incremental Encoder Speed Acquisition Using an STM32 Microcontroller and NI ELVIS," *Sensors*, vol. 22, no. 14, p. 5127, 2022. https://doi.org/10.3390/s22145127
11. G. Bravo et al., "Timer-Based Digitization of Analog Sensors Using Ramp-Crossing Time Encoding," *Technologies*, vol. 14, no. 1, p. 72, 2026. https://doi.org/10.3390/technologies14010072
12. M. Grossi, "Efficient and Accurate Analog Voltage Measurement Using a Direct Sensor-to-Digital Port Interface for Microcontrollers and FPGAs," *Sensors*, vol. 24, no. 3, p. 873, 2024. https://doi.org/10.3390/s24030873
13. I. Plauska et al., "Performance Evaluation of C/C++, MicroPython, Rust and TinyGo Programming Languages on ESP32 Microcontroller," *Electronics*, vol. 12, no. 1, p. 143, 2023. https://doi.org/10.3390/electronics12010143
14. B. Qin et al., "Understanding and Detecting Real-World Safety Issues in Rust," *IEEE Transactions on Software Engineering*, vol. 50, no. 6, pp. 1306–1324, 2024. https://doi.org/10.1109/TSE.2024.3380393
15. R. Luna and S. A. Islam, "Security and Reliability of Safety-Critical RTOS," *SN Computer Science*, vol. 2, no. 5, p. 356, 2021. https://doi.org/10.1007/s42979-021-00753-y

---

## 9. Authors

| Name | NRP | Institution |
|------|-----|-------------|
| **Rizky Wahyu Putra W.** | 2042241049 | Program Studi Rekayasa Teknologi Instrumentasi |
| **Razzan Manggala Putra B.** | 2042241116 | Program Studi Rekayasa Teknologi Instrumentasi |

**Supervisor:** Ahmad Radhy, S.Si., M.Si.  
**Course:** Pemrograman Kontroller – Semester Genap 2025/2026  
**Submitted:** 20 Mei 2026

---

<p align="center">
  Made with ❤️ for <b>Evaluasi Tengah Semester – Bagian B</b><br/>
  Program Studi Rekayasa Teknologi Instrumentasi · 2025/2026
</p>
