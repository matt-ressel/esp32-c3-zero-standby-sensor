# ESP32-C3 Zero-Standby Environmental Sensor

## 🎯 Project Goal

To create a wireless, battery-powered environmental sensor, optimized for extremely low power consumption. The primary goal is to achieve a theoretical battery life exceeding **10 years** on a single Li-SOCl₂ (e.g., FANSO ER18505M 3.6V, 3500mAh) battery.

This project achieves this goal through a unique **"Zero Standby"** architecture, where the microcontroller is completely disconnected from power during its idle state. The duty cycle is managed by an external, nano-power system timer.

---

## 🚀 Key Features & Architecture

*   **Ultra-Low Electronic Quiescent Current:** The total current consumed by the electronics in the power-off state has been meticulously optimized to be under 100 nA. This value is the sum of the quiescent currents (Iq) of all components permanently connected to the battery:
    *   **TPS63900 Buck-Boost (in shutdown):** ~60 nA
    *   **TPL5111 System Timer:** ~35 nA
    *   **SiP32431 Load Switch (in off-state):** ~0.01 nA (practically zero)
    *   **Total Measured Electronic Iq ≈ 95 nA**

    This successfully achieves the design goal of making the electronic footprint in the standby state almost immeasurable. It ensures that the theoretical battery life is maximized, being limited almost exclusively by the energy consumed during the brief active cycles.
*   **Power Gating Architecture:** Instead of using Deep Sleep mode, the TPL5111 completely enables and disables the power supply for the entire system (MCU + sensor), eliminating any MCU-related leakage currents.
*   **Ultra-Fast On-Time:** The entire active cycle (boot, measure, transmit, shutdown) is highly optimized to complete in **well under 100 milliseconds**, which is critical for minimizing energy consumption.
*   **Optimized Operational Sequence:** The firmware uses advanced techniques like **latency hiding**, where long physical processes (like battery measurement stabilization) are executed in the background during other necessary initializations (like the Wi-Fi stack), effectively reducing their time cost to zero.
*   **ESP-NOW Communication:** Utilizes the lightweight and fast ESP-NOW protocol, configured for Long Range (LR) mode, for robust and low-power data transmission.
*   **Professional Component-Based Architecture:** The code is structured into fully independent, reusable components (e.g., `bme280_driver`, `espnow_sender`, `sensor_manager`) in line with the best practices for ESP-IDF, ensuring clean separation of concerns and easy maintenance.

---

## 🛠️ Hardware Components (Bill of Materials)

| Component             | Model/Value                                     | Role                                                     |
| --------------------- | ----------------------------------------------- | -------------------------------------------------------- |
| **Microcontroller**   | Espressif ESP32-C3-MINI-1                       | Main processing unit, RF communication                   |
| **System Timer**        | Texas Instruments TPL5111                       | External watchdog and duty cycle manager (Power Gating)  |
| **Buck-Boost Converter**| Texas Instruments TPS63900                      | High-efficiency 3.3V voltage regulation                |
| **Environmental Sensor**| Bosch BME280                                    | Measures temperature, humidity, and pressure (I²C)       |
| **Battery**             | FANSO ER18505M (Li-SOCl₂, 3.6V, 3500mAh) or similar | Primary power source                                     |
| **Load Switch**         | Vishay SiP32431                                 | Switched voltage divider for V_BATT measurement        |

---

## 🔄 Optimized Software Operational Sequence

The duty cycle is fast, repeatable, and designed for maximum efficiency. The order of operations is critical.

1.  **START (Cold Boot):** The TPL5111 activates the power converter, supplying power to the system. `app_main` begins execution.

2.  **IMMEDIATE CRITICAL ACTIONS:**
    *   **Initialize Critical Drivers:** The bare minimum of drivers needed for immediate measurements are initialized (`bme280_driver`, `battery_monitor`, `power_manager`).
    *   **Start Battery Measurement:** The firmware immediately enables the external voltage divider. This starts the ~25ms physical process of charging the filter capacitor. This process now runs **in the background**.
    *   **Perform BME280 Measurement:** With the battery measurement stabilizing in the background, the CPU **immediately** performs the BME280 sensor reading. This is done as early as possible to get a temperature reading before the CPU core begins to self-heat.

3.  **LATENCY HIDING & NETWORK INIT:**
    *   While the battery measurement capacitor is charging, the firmware productively uses this time to initialize the time-consuming network stack (`wifi_phy_service` and `espnow_sender`). This **hides the 25ms delay**, effectively making the battery stabilization time "free".

4.  **FINALIZE & TRANSMIT:**
    *   Control is passed to the `sensor_manager`.
    *   It **finalizes the battery measurement** by reading the now-stable voltage from the ADC.
    *   It assembles all data (BME280, battery) into a payload, calculates a CRC, and hands it to the `espnow_sender`.
    *   The `espnow_sender` transmits the packet and waits for a hardware-level acknowledgement (ACK) from the receiver.

5.  **SHUTDOWN:**
    *   Upon completion of the transmission (successful or not), a HIGH pulse is generated on the `DONE` pin.
    *   The TPL5111 receives the signal and immediately cuts off power to the system.
    *   A **fail-safe mechanism** is in place: if the shutdown signal fails, the MCU enters its deepest possible sleep state to conserve energy until the TPL5111's own watchdog timer performs a hard power cycle.

---

## ⚙️ Project Setup & Configuration

*   **Framework:** ESP-IDF `v5.5.1` or later.

*   **Configuration:** The project's behavior is configured through a combination of `idf.py menuconfig` and compile-time definitions within the source code.

    *   **Project-Specific Network Settings:** The most important user-configurable parameters (Gateway MAC Address, ESP-NOW Keys, Wi-Fi Channel) are conveniently located in a custom menu within `menuconfig`, named **`ESP32C3 Zero Standby Sensor Configuration`**.

    *   **Performance & System Settings:** Key low-level optimizations are configured in various standard ESP-IDF menus. For example:
        *   CPU Frequency is set under `Component config ---> ESP System Settings`.
        *   Bootloader logging level is set under `Bootloader config`.

    *   **Hardware Pin Configuration:** For direct control and simplicity, hardware pin assignments for peripherals (like I2C for the BME280 and ADC for battery monitoring) are defined as macros at the top of their respective driver files (e.g., in `bme280_driver.c` and `battery_monitor.c`).

    *   **Key `sdkconfig` options for performance:**
        *   `CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y` (Disables bootloader logs for faster startup)
        *   `CONFIG_ESP_SYSTEM_CPU_FREQ_80M=y` (Reduces power consumption)

    *   **In-Code Optimizations:** Other critical optimizations, such as enabling Wi-Fi Long Range mode (`WIFI_PROTOCOL_LR`) and setting the bandwidth to 20MHz, are hard-coded directly in the `wifi_phy_service` component to ensure optimal performance.