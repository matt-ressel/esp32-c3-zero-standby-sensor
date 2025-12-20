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
    *   **Total Electronic Iq ≈ 95 nA**

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
    *   While the battery measurement capacitor is charging, the firmware productively uses this time to initialize the time-consuming network stack (`wifi_service` and `espnow_sender`). This **hides the 25ms delay**, effectively making the battery stabilization time "free".

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


## ⚙️ Getting Started & Configuration

This project is designed for rapid setup using the provided configuration files, while still requiring user-specific network details.

### 1. Mandatory User Configuration (menuconfig)

This repository includes a pre-configured `sdkconfig` file containing all the performance optimizations listed below. This saves you the time of setting them manually.

**However, you MUST configure your own network-specific parameters before building and flashing.**

To do this, run `idf.py menuconfig` and navigate to the following menu:

`---> ESP32C3 Zero Standby Sensor Configuration`

Inside this menu, you must set the following values to match your receiver/gateway:

*   **`Gateway MAC Address`**: The MAC address of your ESP-NOW receiver.
*   **`ESPNOW primary master key (PMK)`**: A 16-character string shared with the receiver.
*   **`ESPNOW local master key (LMK)`**: A 16-character string shared with the receiver for this specific device.
*   **`ESPNOW Channel`**: The Wi-Fi channel (1-13) on which both devices will communicate.
*   **`I2C Pin Configuration`**: Specify the GPIO pins used by the I2C master for the BME280.


### 2. Summary of Key Performance Optimizations

For reference, here is a detailed list of the critical optimizations applied in the provided `sdkconfig` to achieve an ultra-fast boot time (cold boot to shutdown in < 100ms).

The following options, configured via `idf.py menuconfig`, are essential for minimizing boot time and runtime power consumption.

#### Compiler Optimization
For maximum performance, both the main application and the bootloader should be compiled with `-O2` optimization.
*   **Application Optimization Level:**
    *   `Compiler options ---> Optimization Level ---> Optimize for performance (-O2)`

*   **Bootloader Optimization Level:**
    *   `Bootloader config ---> Bootloader optimization level ---> Optimize for performance (-O2)`

#### Bootloader & Logging & System
*   **Skip Image Validation:** This is a major optimization. It instructs the bootloader to skip the time-consuming SHA-256 validation of the application image on every boot.
    *   `Bootloader config ---> [*] Skip image validation always`

*   **Disable Bootloader Log:** Drastically speeds up the initial boot phase.
    *   `Bootloader config ---> Log ---> Bootloader log verbosity ---> No output`

*   **Disable Application Log:** Frees up CPU cycles and reduces on-time. Set to "Info" for debugging and "None" for production.
    *   `Component config ---> Log ---> Log Level ---> Default log verbosity ---> No output`

*   **FreeRTOS:** The project is configured for a single core and a 1000Hz tick rate.
    *   `Component config ---> FreeRTOS ---> Kernel ---> configTICK_RATE_HZ (1000)`

#### ADC & RTC Optimizations
*   **Place ADC ISR in IRAM:** Improves performance of ADC reads by running the code from faster RAM instead of flash.
    *   `Component config ---> ADC ---> [*] Place ISR version ADC oneshot mode read function into IRAM`

*   **Skip RTC Clock Calibration:** Reduces boot time by skipping a time-consuming calibration step.
    *   `Component config ---> Hardware Settings ---> RTC Clock Config ---> (0) Number of cycles for RTC_SLOW_CLK calibration`    

#### Wi-Fi, PHY & LWIP Stack Reduction
The goal is to strip down the network stack to the bare minimum required for ESP-NOW.
*   **Reduce Max TX Power:** A critical setting to reduce peak current draw and stay within the limits of the power supply and battery. 18dBm is a safe starting point.
    *   `Component config ---> Wi-Fi ---> (18) Max WiFi TX power (dBm)`
    
*   **Enable Brownout Power Reduction:** A fail-safe mechanism that reduces Tx power on the next boot if a brownout occurs.
    *   `Component config ---> PHY ---> [*] Reduce PHY TX power when brownout reset`

*   **Disable Unused Wi-Fi Features:** ESP-NOW does not require SoftAP, WPA3, Enterprise security, or OWE. Disabling these significantly reduces the final binary size and memory footprint.
    *   `Component config ---> Wi-Fi ---> [ ] WiFi SoftAP Support`
    *   `Component config ---> Wi-Fi ---> Security features ---> [ ] Enable WPA3-Personal`
    *   And other similar options...

*   **Disable LWIP (TCP/IP Stack):** ESP-NOW operates at the MAC layer and does not use the TCP/IP stack (LWIP). Disabling IPv6 and DHCP server further reduces code size and memory usage.
    *   `Component config ---> LWIP ---> [ ] Enable IPv6`
    *   `Component config ---> LWIP ---> [ ] DHCPS: Enable IPv4 Dynamic Host Configuration Protocol Server (DHCPS)`

#### Flash & PHY
*   **SPI Flash Configuration:** Using the fastest possible mode and frequency reduces the time spent loading the application from flash.
    *   `Serial flasher config ---> Flash SPI mode ---> QIO`
    *   `Serial flasher config ---> Flash SPI speed ---> 80 MHz`
    
*   **Disable PHY Calibration:** Skips Wi-Fi/Bluetooth PHY calibration on boot, saving a significant amount of time. This is safe as calibration data is stored in NVS and loaded on first boot.
    *   `Component config ---> PHY ---> [*] Store phy calibration data in NVS`
    *   `Component config ---> PHY ---> Calibration mode (Calibration none)`
 
### 3. Boot ROM Log Disable (eFuse) - CRITICAL STEP

By default, the ESP32-C3's internal Boot ROM prints diagnostic messages to the UART during the very first moments of startup. This adds a significant delay. Disabling this is a **one-way, irreversible operation** that requires burning eFuses.

**Warning: Burning eFuses is permanent. Proceed with caution.**

To achieve the fastest possible boot time, connect the device and run the following commands using `espefuse.py`:

```bash
# Disable ROM logs for UART0
espefuse.py --port YOUR_PORT burn_efuse UART_PRINT_CONTROL 3

# For ESP32-C3 with built-in USB-JTAG, also disable its ROM logs
espefuse.py --port YOUR_PORT burn_efuse DIS_USB_SERIAL_JTAG_ROM_PRINT 1    

```
