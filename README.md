# ⚙️ Final FreeRTOS Project

---

## 📑 Table of Contents
- [Project Overview](#project-overview)
- [Author & Contact](#author--contact)
- [Repository Structure](#repository-structure)
- [Technologies & Tools](#technologies--tools)
- [Key Features](#key-features)
- [Firmware Analysis (High Level)](#firmware-analysis-high-level)
- [Hardware Integration & Wiring](#hardware-integration--wiring)
- [Build, Flash & Run](#build-flash--run)
- [Testing & Verification](#testing--verification)
- [Troubleshooting Guide](#troubleshooting-guide)
- [Future Improvements](#future-improvements)
- [Demo / Screenshots](#demo--screenshots)
- [License](#license)

---

## Project Overview
This project demonstrates embedded firmware built on **FreeRTOS** for a microcontroller-based device with Bluetooth and sensor integration. The firmware coordinates multiple concurrent tasks, handles peripheral I/O, and communicates with external devices.

---

## Author & Contact
- **Author:** Thành Tài  
- **GitHub:** [Thanhtai1305](https://github.com/Thanhtai1305)  


---

## Repository Structure
- `Images/` — wiring diagrams, photos, screenshots  
- `Linh kiện/` — component list (schematics, part numbers)  
- `bluetooth___sumo/` — primary firmware folder (contains `.ino` and support files)  
- `TỔNG QUAN.txt` — project overview / design notes  

---

## Technologies & Tools
- **RTOS:** FreeRTOS  
- **Language:** Embedded C / C++ (`.ino` Arduino-style source)  
- **Toolchains:** Arduino IDE / PlatformIO / STM32CubeIDE (depending on MCU)  
- **Peripherals:** Bluetooth (UART), sensors (I2C/SPI/ADC), LEDs, buttons  

---

## Key Features
- Preemptive multitasking with FreeRTOS  
- Wireless communication via Bluetooth  
- Sensor sampling and real-time data handling  
- Modular task-based architecture for maintainability  

---

## Firmware Analysis (High Level)
> ⚠️ This section will be expanded once `.ino` source code is fully analyzed.  
> Paste your `.ino` content here and the README will be updated with concrete values.

### Task Summary (example)
| Task Name      | Function                  | Priority | Stack Size | Frequency | Notes                      |
|----------------|---------------------------|---------:|-----------:|----------:|----------------------------|
| Task_Sensor    | Reads sensor data         | 3        | 1024       | 100 ms   | Uses mutex for bus access  |
| Task_Bluetooth | Send/receive BT messages  | 2        | 1024       | Event    | UART Rx interrupt-driven   |
| Task_Logger    | Serial logging / debugging| 1        | 512        | 1 s      | Optional                   |

### Inter-task Communication
- Queues for data exchange  
- Mutexes for shared resources  
- ISRs for UART / external interrupts  

---

## Hardware Integration & Wiring

### Pinout (placeholder)
| Signal        | MCU Pin | Peripheral |
|---------------|---------|------------|
| Bluetooth TX  | D2      | HC-05 RX   |
| Bluetooth RX  | D3      | HC-05 TX   |
| LED Status    | D13     | Indicator  |
| Sensor SDA    | A4      | I2C SDA    |
| Sensor SCL    | A5      | I2C SCL    |

> Replace with actual pins from `.ino`

### Bill of Materials
- MCU board: STM32 / ESP32 / Arduino (depending on target)  
- Bluetooth module: HC-05 / HC-06 / BLE module  
- Sensors: e.g., MPU6050, DHT22 (update based on `.ino`)  
- Power: 5V/3.3V regulated  

---

## Build, Flash & Run

### Arduino IDE
1. Open `.ino` file in Arduino IDE  
2. Select **Board** and **Port**  
3. Click **Upload**  

### PlatformIO (recommended)
```bash
pio run -e <environment>        # compile
pio run -e <environment> --target upload   # flash
pio device monitor -b 115200    # serial monitor
```

### Flashing Notes
- Serial monitor default: **115200 baud, 8N1**  
- ESP32: use `esptool.py` if needed  
- STM32: use ST-Link or DFU  

---

## Testing & Verification
1. Power on and check **status LED**  
2. Confirm FreeRTOS scheduler started (serial log)  
3. Send Bluetooth commands and validate response  
4. Trigger sensor inputs and check system reaction  

---

## Troubleshooting Guide
- **HardFault / reset** → Increase stack size for crashing task  
- **Bluetooth not connecting** → Check RX/TX wiring and baud rate  
- **ISR crash** → Ensure only `FromISR` FreeRTOS APIs are used  

---

## Future Improvements
- 📈 Add more sensors for extended applications  
- 🔄 Implement low-power / energy-saving modes  
- 🔒 Add encryption for Bluetooth communication  
- 🌐 Expand to IoT with cloud integration  

---

## Demo / Screenshots
- Add photos or diagrams under `Images/`  
- Insert demo video link here (e.g., YouTube)  

---

## License
This project is developed for **educational purposes only**.  
*(Add open-source license file if required)*  
