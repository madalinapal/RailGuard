# RailGuard

**RailGuard** is an automated railway crossing management system. Built on **FreeRTOS** for Arduino, the project ensures a high-level of safety and operational efficiency by synchronizing train detection with barrier actuation in real-time.

---

##  Overview
The system manages a level crossing using a Finite State Machine (FSM) approach. It balances two critical constraints:
1.  **Safety:** Keeping the barrier closed during train transit.
2.  **Utility:** Releasing road traffic if a train is idling at the crossing for too long.

### Key Logic & Constraints
* **Safety Window ($\tau_{min}$):** Once a train is detected, the barrier stays closed for at least **1 second** to prevent "flickering" or premature opening.
* **Utility Timeout ($\tau_{max}$):** If a train remains in the detection zone for more than **5 seconds**, the system forces the barrier open to prevent traffic jams.
* **Linear Motion:** The barrier doesn't "snap" into position; it moves smoothly over **30ms** using a linear interpolation algorithm.

---

##  Hardware Configuration
* **MCU:** Arduino Mega 2560 (or compatible)
* **Sensors:** * `S1 (Approach)`: HC-SR04 Ultrasonic (Trig: 8, Echo: 7)
    * `S2 (Departure)`: HC-SR04 Ultrasonic (Trig: 10, Echo: 6)
* **Actuator:** Servo Motor (Signal: Pin 9)

---

##  Software Architecture
The project leverages the power of **FreeRTOS** to handle concurrency through three prioritized tasks:

| Task | Priority | Description |
| :--- | :---: | :--- |
| **`taskControl`** | 3 (High) | The "Brain". Processes events from the queue and manages timers. |
| **`taskServo`** | 2 (Med) | The "Muscle". Executes smooth linear movement commands. |
| **`taskSensors`** | 1 (Low) | The "Eyes". Periodically polls ultrasonic sensors and debounces signals. |

### Communication Tools
* **Queues:** Used for inter-task communication (Events & Servo Commands).
* **Software Timers:** Manage $\tau_{min}$, $\tau_{max}$, and movement confirmations without blocking the CPU.

---

##  Installation & Usage
1.  **Cloning:** Download the source code.
2.  **Dependencies:** Ensure you have the `Arduino_FreeRTOS` and `Servo` libraries installed in your IDE.
3.  **Wiring:** Connect the hardware according to the `PIN` definitions in the code.
4.  **Monitoring:** Open the Serial Monitor at **9600 baud** to see the system logs:
    > `S1 < 5cm -> INCHID bariera si respect TAU_MIN=1s.`  
    > `--- Barieră CLOSED complet (30ms).`
