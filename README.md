# ESP32-C3 Zero-Standby Environmental Sensor

## 🎯 Project Goal

To create a wireless, battery-powered environmental sensor, optimized for extremely low power consumption. The primary goal is to achieve a theoretical battery life exceeding **10 years** on a single Li-SOCl₂ (FANSO-ER18505M 3.6V) battery.

This project achieves this goal through a unique **"Zero Standby"** architecture, where the microcontroller is completely disconnected from power during its idle state. The duty cycle is managed by an external, nano-power system timer.

---

## 🚀 Key Features & Architecture

*   **Extremely Low Quiescent Current:** The standby current draw is dominated by the TPL5111's quiescent current and the battery's self-discharge rate (theoretically <100 nA).
*   **Power Gating Architecture:** Instead of using Deep Sleep mode, the TPL5111 completely enables and disables the power supply for the entire system (MCU + sensor), eliminating any MCU-related leakage currents.
*   **Ultra-Fast On-Time:** The entire active cycle (boot, measure, transmit, shutdown) is optimized to complete in **under 100 milliseconds**, which is critical for minimizing energy consumption.
*   **ESP-NOW Communication:** Utilizes the lightweight and fast ESP-NOW protocol for data transmission, drastically reducing the radio activity time compared to a traditional Wi-Fi connection.
*   **Precise Power Management:** Employs a high-efficiency buck-boost converter (TPS63900) and a hardware-based input current limit to protect the high-internal-impedance battery.

---

## 🛠️ Hardware Components (Bill of Materials)

| Component             | Model/Value                                     | Role                                                     |
| --------------------- | ----------------------------------------------- | -------------------------------------------------------- |
| **Microcontroller**   | Espressif ESP32-C3-MINI-1                       | Main processing unit, RF communication                   |
| **System Timer**        | Texas Instruments TPL5111                       | External watchdog and duty cycle manager (Power Gating)  |
| **Buck-Boost Converter**| Texas Instruments TPS63900                      | High-efficiency 3.3V voltage regulation                |
| **Environmental Sensor**| Bosch BME280                                    | Measures temperature, humidity, and pressure (I²C)       |
| **Battery**             | FANSO-ER18505M(Li-SOCl₂, 3.6V, 3500mAh) or similar | Primary power source                                     |
| **Load Switch**         | Vishay SiP32431                                 | Switched voltage divider for V_BATT measurement        |

---

## 🔄 Software Operational Sequence

The duty cycle is simple, fast, and repeatable, occurring every ~20 minutes (configurable via a resistor on the TPL5111).

1.  **START (Cold Boot):** The TPL5111 activates the power converter, supplying power to the system.
2.  **INITIALIZATION:** Configuration of necessary GPIO pins (`DONE`, `ADC_EN`). Bootloader logs are disabled to maximize startup speed.
3.  **SENSOR MEASUREMENT:** Immediate data readout from the BME280 via I²C before the ESP32 chip has a chance to heat up.
4.  **BATTERY VOLTAGE MEASUREMENT:**
    *   Enable the external voltage divider (GPIO `ADC_EN` -> HIGH).
    *   Short pause (~25ms) for voltage stabilization on the filter capacitor.
    *   Perform ADC measurement (with averaging).
    *   Immediately disable the divider (GPIO `ADC_EN` -> LOW) to prevent energy drain.
5.  **DATA TRANSMISSION:**
    *   Initialize Wi-Fi in a minimal mode and the ESP-NOW protocol.
    *   Send a single packet with the measurement data.
6.  **SHUTDOWN:**
    *   Upon successful transmission confirmation (via callback), generate a short HIGH pulse on the `DONE` pin.
    *   The TPL5111 receives the signal and immediately cuts off power to the system.

---

## ⚙️ Project Setup & Configuration

*   **Framework:** ESP-IDF `v5.5.0`

*   **Configuration:** The project is configured for minimal power consumption. Key settings in `sdkconfig` include:
    *   `CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y`
    *   `CONFIG_ESP_MAIN_TASK_STACK_SIZE=2048`
    *   `CONFIG_ESP_SYSTEM_CPU_FREQ_80M=y`
    *   ... (further instructions to be added)