# Bluetooth Remote-Controlled Obstacle-Avoiding Car System 🚗💨

> A robust embedded system project based on PIC18F4520, featuring dual-priority interrupt handling, fail-safe collision prevention, and real-time remote control.

## 📖 Overview

This project is a microcomputer system design that integrates **UART communication**, **PWM motor control**, and **ultrasonic sensing** to create a semi-autonomous vehicle. The system allows users to control the vehicle via a Bluetooth mobile app while an onboard safety supervisor monitors environmental data to prevent collisions.

The core logic is implemented on a **PIC18F4520** microcontroller, utilizing hardware timers and a custom Finite State Machine (FSM) to handle concurrent tasks without blocking the main loop.

## ✨ Key Features

### 1. Remote Control & Locomotion

* **Bluetooth Connectivity:** Receives `F` (Forward), `B` (Backward), `L` (Left), `R` (Right), and `S` (Stop) commands via HC-05 module using UART protocol.


* **Precision Motor Driving:** Controls DC motors via L298N driver and a Servo motor for directional/interactive components.



### 2. Intelligent Safety & Feedback (Fail-Safe)

* **Active Collision Prevention:** The system automatically overrides "Forward" commands when an obstacle is detected within **20cm**.


* **Auto-Recovery Mode:** If the vehicle remains in a danger zone (<20cm) for more than **5 seconds**, it automatically reverses to a safe distance to prevent hardware damage.


* **Audible & Visual Feedback:**
* **TM1637 Display:** Shows real-time distance in cm.


* **Dynamic Buzzer:** Beep frequency increases as the vehicle approaches obstacles (<60cm: slow, <40cm: medium, <20cm: rapid).





### 3. Interactive Elements

* **Laser & Servo Action:** Special command `H` triggers a laser module and a synchronized servo "tail-wagging" sequence for demonstration purposes.



## 🛠 System Architecture

### Hardware Components

| Component | Description | Connection |
| --- | --- | --- |
| **MCU** | Microchip PIC18F4520 | Core Controller |
| **Driver** | L298N Dual H-Bridge | PORTB (RB0-RB3) 

 |
| **Sensor** | HC-SR04 Ultrasonic | TRIG: RD2, ECHO: RD3 

 |
| **Comms** | HC-05 Bluetooth Module | UART (TX/RX) 

 |
| **Display** | TM1637 4-Digit 7-Seg | CLK: RD4, DIO: RD5 

 |
| **Actuator** | SG90 Servo Motor | RC2 (CCP1 PWM) 

 |
| **Output** | Active Buzzer | RD1 

 |
| **Output** | Laser Module | RC5 

 |

> *Note: Please refer to the full circuit diagram in the `docs/` folder.* (建議你把 PDF 第 2 頁的電路圖截圖放進去)

### Tech Stack & Tools

* **Language:** Embedded C
* **IDE:** MPLAB X IDE 


* **Compiler:** XC8 Compiler 


* **Hardware Debugger:** PICkit 3/4 



## 🧩 Technical Highlights & Challenges

### 1. Interrupt Priority Management

**Challenge:** The system initially suffered from instability where the high-frequency Timer interrupts (for Buzzer/PWM) interfered with the UART communication, causing dropped packets.
**Solution:**

* Implemented **Dual-Priority Interrupts**:
* **High Priority (ISR_H):** Handles critical UART reception and ADC conversion to ensure zero command latency.


* **Low Priority (ISR_L):** Handles Timer1 overflow for non-critical buzzer toggling.





### 2. Custom Bit-Banging Driver

**Challenge:** Integrating the TM1637 display required precise timing that conflicted with standard libraries during interrupts.
**Solution:**

* Wrote a custom **Bit-Banging** protocol for the TM1637.
* Added logic to temporarily disable global interrupts during critical data transmission segments to prevent timing violations and display corruption.



### 3. Non-Blocking State Machines

**Challenge:** Using `delay()` for the buzzer or safety checks would freeze the CPU, making the car unresponsive to stop commands.
**Solution:**

* Designed a **Time-Tick based State Machine** (using `system_time_ms`).
* The buzzer logic checks timestamps to toggle states (`BUZZ_IDLE`, `BUZZ_ON`, `BUZZ_OFF`) without halting the main loop execution.



## 🚀 How to Build & Run

1. **Clone the repo:**
```bash
git clone https://github.com/your-username/your-repo-name.git

```


2. **Open Project:**
* Launch **MPLAB X IDE**.
* Open the project folder.


3. **Hardware Setup:**
* Connect the PIC18F4520 and peripherals according to the pin definitions in `main.c`.
* Ensure the L298N is powered by an external battery source (not USB).


4. **Flash:**
* Connect PICkit programmer.
* Build and Run (Make and Program Device).


5. **Control:**
* Pair your Android phone with the HC-05 module (Default PIN: 1234).
* Use a Bluetooth Serial Terminal app to send characters: `F`, `B`, `L`, `R`, `S`.



## 📂 Project Structure

```
.
├── sourceCode.c
└── README.md           # Project documentation

```

## 👥 Contributors

* **Peng Yi Cheng** (Hardware Integration, System Architecture) 


* **Wang Jyun Kai** (Motor Control Driver) 


* **Huang Kai hsaun** (Ultrasonic & Display Logic) 


* **Tsia Yuan Chin** (UART Communication & Safety Logic) 



---

*Created as a Final Project for the Microcomputer Systems Course at NCKU.* 
