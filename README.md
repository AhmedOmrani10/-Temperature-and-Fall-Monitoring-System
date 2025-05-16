# Temperature and Fall Monitoring with STM32

<div align="center">
  <img src="https://github.com/user-attachments/assets/911cd09c-1010-426b-a4fd-07ae26a8f1bd" alt="System Overview" width="600"/>
  <p><em>Figure: System overview showing STM32-based temperature and fall monitoring with alarm triggers.</em></p>
</div>

## Overview
This project involves the development of an embedded system using an STM32 microcontroller that performs the following key functions:
- **Temperature Monitoring:** Continuously reads ambient temperature and checks for threshold violations.
- **Fall Detection:** Detects and reports device falls using accelerometer data.
- **Alarm System:** Triggers an audible alarm when temperature thresholds are exceeded or a fall is detected.
- **Multitasking with FreeRTOS:** All functionalities are handled using real-time tasks for efficient and responsive system behavior.

The system is designed for environments where safety and monitoring are essential, such as healthcare, industrial control, and personal safety devices.

## Features
- 📉 **Fall Detection:** Detects rapid orientation changes indicating a fall.
- 🌡️ **Temperature Monitoring:** Reads temperature using analog sensors via ADC.
- 🚨 **Alarm Trigger:** Buzzer alert system activated based on event thresholds.
- ⏱️ **Real-Time Scheduling:** FreeRTOS manages tasks like sensor reading, data processing, and alert handling.
- 🧪 **Simulation Support:** Developed and tested with a C++ and OpenGL-based simulation framework.

## System Architecture
- **Microcontroller:** STM32F411
- **Sensors:** ADXL345, LM75B
- **RTOS:** FreeRTOS for task scheduling and synchronization




## Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/stm32-temp-fall-monitor.git
   cd stm32-temp-fall-monitor
