# Motor Cooling Sleeve DAQ & Thermal Characterization System for Formula SAE

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![Arduino](https://img.shields.io/badge/Hardware-ATmega2560-00979D.svg)](https://www.arduino.cc/)
[![Architecture](https://img.shields.io/badge/Architecture-Ratiometric%20ADC-orange.svg)](#circuit-design--ratiometric-sensing)

An end-to-end DAQ system for testing electric motor liquid cooling sleeves under high thermal loads. The system couples an **ATmega2560 (Elegoo Mega 2560 R3)** microcontroller with an asynchronous, multi-threaded **Python** monitoring engine featuring real-time terminal telemetry, dynamic conduction compensation, and automated logging.

---

## System Overview

```
+-------------------------------------------------------------+
|                     HARDWARE LAYER                          |
|                                                             |
|   [ Koolance Flowmeter (Hall Effect) ] -> INT4 (Pin 2)      |
|   [ Inline Coolant NTC Thermistors   ] -> ADC (A0, A1)      |
|   [ 8x Stator Edge NTC Thermistors   ] -> ADC (A2 - A9)     |
|                               |                             |
|               Elegoo Mega 2560 R3 (ATmega2560)              |
|        Non-blocking 20 Hz Sampling & Ratiometric ADC        |
+------------------------------+------------------------------+
                               | USB Serial (115200 Baud JSON)
                               v
+-------------------------------------------------------------+
|                      SOFTWARE LAYER                         |
|                                                             |
|   1. Serial Parser & Auto-Discovery (`pyserial`)            |
|   2. Physical Calibration & 1D Conduction Offset            |
|   3. Hydraulic & Thermodynamic Processing:                  |
|      - Flow Rate (LPM) & Pulse Debouncing                   |
|      - Heat Absorption Rate Q (W) & Thermal Resistance R_th |
|      - Multi-channel Rate of Change dT/dt                   |
|   4. Terminal Dashboard (Unicode Braille Graphs via `rich`) |
|   5. Automated CSV Export with Monotonic Timestamps         |
+-------------------------------------------------------------+
```
---
## Hardware Specifications
- **Data Acquisition:** Elegoo Mega 2560 R3 (16 MHz AVR, multi-channel 10-bit ADC)
- **Stator Core Emulation:** Custom CNC-machined 6061-T6 aluminum dummy motor core, precision-bored for internal cartridge heater arrays and sub-surface instrumentation
- **Test Articles:** Interchangeable jacket prototypes featuring diverse internal cooling channel architectures
- **Thermal Instrumentation:** 
  - 8x sub-surface 10k NTC thermistors embedded directly at the core-jacket interface
  - 2x Koolance G1/4" inline thermistor probes for differential fluid delta-T ($\Delta T$)
- **Hydraulics & Flow Monitoring:** Koolance Hall-effect turbine flowmeter paired with a closed-loop centrifugal pump and external heat exchanger

---

## Key Engineering Highlights

- **Ratiometric ADC Measurement:** Pull-up resistors and the ATmega2560 ADC share the 5V excitation rail, mathematically canceling supply voltage fluctuations from USB ports without requiring external precision voltage references.
- **Dynamic 1D Fourier Thermal Conduction Offset:** Embedded thermistors are situated $0.095\text{ in}$ beneath the outer stator surface. The processing pipeline dynamically computes heat flux ($Q$) and offsets internal sensor temperatures using cylindrical Fourier conduction resistance to yield accurate surface boundary temperatures in real time.
- **Hardware Interrupt Debouncing:** Hall-effect flow sensor pulses trigger INT4 on Pin 2 with a $5\text{ ms}$ ISR lockout window, suppressing electrical noise and mechanical contact chatter.
- **Zero-Dependency CLI Telemetry:** Custom Unicode Braille multi-series live graph rendered in the terminal dashboard displaying inlet/outlet coolant temperatures, motor average, and hottest sensor profiles without GUI overhead.
- **Mock / Simulation Mode:** Integrated software simulator (`--mock`) enables full visualization, calibration validation, and data pipeline testing without physical hardware attached.

---

## Thermal & Hydraulic Modeling

### 1. Ratiometric Thermistor Conversion
Using the Steinhart-Hart / Beta equation with excitation voltage cancellation:
$$R_{\text{ntc}} = R_{\text{fixed}} \cdot \frac{\text{ADC}}{1023 - \text{ADC}}$$
$$T = \left(\frac{1}{T_0} + \frac{1}{\beta}\ln\left(\frac{R_{\text{ntc}}}{R_0}\right)\right)^{-1} - 273.15$$

### 2. Coolant Heat Absorption Rate ($Q$)
Real-time thermal energy absorbed by the cooling fluid:
$$Q = \dot{m} \cdot C_p \cdot (T_{\text{out}} - T_{\text{in}}) = \left(\frac{\text{LPM}}{60}\right) \cdot \rho_{\text{water}} \cdot C_{p,\text{water}} \cdot (T_{\text{out}} - T_{\text{in}})$$

### 3. Conduction Correction (Cylindrical 1D Wall)
To extract true jacket interface temperatures from embedded sensors ($d = 0.095\text{ in}$ from surface, $k_{\text{Al}} = 167\text{ W}/(\text{m}\cdot\text{K})$):
$$R_{\text{cond}} = \frac{\ln\left(\frac{r_{\text{outer}}}{r_{\text{outer}} - d}\right)}{2 \pi k_{\text{Al}} L}$$
$$T_{\text{surface}} = T_{\text{measured}} - (Q_{\text{smooth}} \cdot R_{\text{cond}})$$

### 4. Overall Thermal Resistance ($R_{\text{th}}$)
$$R_{\text{th}} = \frac{T_{\text{motor, avg}} - T_{\text{coolant, in}}}{Q} \quad [^\circ\text{C}/\text{W}]$$

---

## Hardware Specification & Pinout

### Analog Sensor Mapping (Voltage Divider to 5V Rail)

```
Mega 5V ─────[ R_FIXED (nominal 9820 Ω) ]─────┬───── Analog Pin (A0 - A9)
                                              │
                                      [ NTC Thermistor ]
                                              │
                                           Mega GND
```

| Channel | Pin | Description | Transducer / Part | Nominal $R_0$ | $\beta_{25/85}$ |
| :--- | :---: | :--- | :--- | :---: | :---: |
| **Coolant In** | `A0` | Fluid Inlet Temperature | Koolance Inline G1/4 | $10\,\text{k}\Omega$ @ 25°C | $3435\,\text{K}$ |
| **Coolant Out** | `A1` | Fluid Outlet Temperature | Koolance Inline G1/4 | $10\,\text{k}\Omega$ @ 25°C | $3435\,\text{K}$ |
| **Motor T1–T8**| `A2`–`A9` | 8-Point Stator Radial Array | Ametherm PANE 103395-410 | $10\,\text{k}\Omega$ @ 25°C | $3950\,\text{K}$ |
| **Expansion** | `A10`–`A15`| Auxiliary Thermal Channels | Reserved | — | — |

### Digital Frequency Sensor

| Signal | Pin | Interrupt | Mode | Function | Calibration |
| :--- | :---: | :---: | :---: | :--- | :--- |
| **Flow Sensor** | `Pin 2` | `INT4` | `INPUT_PULLUP` | Hall-effect pulse counting | $0.308\,\text{LPM / Hz}$ ($5\text{ ms}$ debounce) |

---


## Getting Started

### Prerequisites
- Python 3.10+
- Arduino IDE or CLI (with AVR core installed)
- Hardware: Elegoo / Arduino Mega 2560 R3 with USB connection

### 1. Firmware Installation
1. Connect the ATmega2560 board to your host computer via USB.
2. Open [`sleeve_daq/sleeve_daq.ino`](sleeve_daq/sleeve_daq.ino) in Arduino IDE.
3. Select **Board**: `Arduino Mega or Mega 2560` and choose the corresponding serial port.
4. Upload sketch (`Ctrl+U` / `Cmd+U`).

### 2. Python Environment Setup
```bash
# Clone the repository
git clone https://github.com/ManagerCat/sleeve-testing-daq.git
cd sleeve-testing-daq

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install runtime dependencies
pip install -r requirements.txt
```

### 3. Launching the Logger

**Live Production Run:**
```bash
python3 sleeve_logger.py
```
*The application automatically resolves the microcontroller serial port, initiates handshake verification, and opens the live console display.*

**Hardware-in-the-Loop Mock Simulation:**
```bash
python3 sleeve_logger.py --mock
```
*Allows end-to-end evaluation of data calculation pipelines, Braille dashboard rendering, and file output without physical hardware attached.*

---

## Configuration (`config.toml`)

All hardware settings, geometry, fluid constants, and sensor calibrations are cleanly decoupled into [`config.toml`](config.toml):

```toml
[hardware]
serial_port = "AUTO"         # Auto-scans for Mega 2560 or specify exact port
baud_rate = 115200

[circuit]
default_r_fixed = 9820.0     # Nominal pull-up resistor (Ohms)

# Individual measured resistor values per channel for calibration:
[circuit.resistor_ohms]
water_in = 9820.0
water_out = 9815.0
T1 = 9825.0
# ...

[motor_geometry]
enabled = true
distance_from_edge_in = 0.095 # Depth of embedded thermistors (inches)
outer_diameter_in = 3.778     # Outer diameter of test motor (inches)
length_in = 3.6               # Active jacket contact length (inches)
k_aluminum = 167.0            # Thermal conductivity (W/(m*K))
```

---

## Output Data Format

Tests automatically export timestamped CSV files directly to `~/Downloads/sleeve_test_<YYYYMMDD_HHMMSS>.csv`.

| Field Name | Unit | Description |
| :--- | :---: | :--- |
| `timestamp` | ISO-8601 | Absolute system date and time |
| `elapsed_s` | $\text{s}$ | Monotonic test execution timer |
| `water_in_C` / `water_out_C` | $^\circ\text{C}$ | Coolant fluid boundary temperatures |
| `delta_water_C` | $^\circ\text{C}$ | Temperature rise across the jacket |
| `flow_LPM` | $\text{LPM}$ | Calibrated volumetric coolant flow rate |
| `watts_smoothed` | $\text{W}$ | Rolling-averaged fluid thermal dissipation |
| `edge_offset_C` | $^\circ\text{C}$ | Dynamic 1D conduction offset ($Q \cdot R_{\text{cond}}$) |
| `motor_1_C` – `motor_8_C` | $^\circ\text{C}$ | Surface-corrected motor edge temperature distribution |
| `motor_spread_C` | $^\circ\text{C}$ | Spatial temperature gradient across circumference ($T_{\text{max}} - T_{\text{min}}$) |
| `motor_rate_C_per_min` | $^\circ\text{C}/\text{min}$ | Motor thermal transient rate of change ($\frac{dT}{dt}$) |
| `hotspot_sensor` | ID | Sensor channel indicating maximum temperature |
| `thermal_resistance_C_per_W` | $^\circ\text{C}/\text{W}$ | Measured jacket thermal impedance |