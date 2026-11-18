# Smart Traffic Junction Controller 🚦

**IE3064: Embedded Systems Engineering | Year 3 Semester 1 (2026)**  
**Institution:** Sri Lanka Institute of Information Technology (SLIIT)

---

## 📖 Project Overview

The **Smart Traffic Junction Controller** is a bare-metal embedded system built around an **ATmega328P** microcontroller. The project integrates multi-phase vehicle signal timing, an interrupt-driven pedestrian push-button crossing mechanism, a 7-segment countdown display, and an LDR-based ambient light sensor for automated night-mode operation.

The system was fully modeled and simulated in **Proteus Design Suite**, then physically prototyped on a hardware testbed.

---

## ⚡ Key Features

- **Bare-Metal AVR Assembly Firmware:** Written entirely in AVR Assembly using Microchip Studio for direct port manipulation, timer control, and ADC operations.
- **8-State Finite State Machine (FSM):** Manages synchronized North/South and East/West traffic phases, yellow warning transitions, and mandatory all-red clearance intervals.
- **Deferred Pedestrian Crosswalk:** Detects pedestrian requests through the `INT0` external interrupt on `PD2`, safely queues the request until the current phase and all-red clearance complete, then starts an active 5-to-0 countdown on a 7-segment display.
- **Automated Night Mode with Hysteresis:** Samples ambient light through an LDR voltage divider on `PC2 (ADC2)`. Low-light conditions activate simultaneous yellow flashing at 1 Hz, while a strict 5-second continuous light-recovery period is required before normal cycling resumes.

---

## 🛠️ Hardware Component Stack

| Component | Quantity | Specification / Function |
| :--- | :---: | :--- |
| **Microcontroller** | 1 | ATmega328P with a 16 MHz external crystal oscillator |
| **Traffic LEDs** | 12 | 4 red, 4 yellow, and 4 green LEDs grouped into North/South and East/West pairs |
| **Pedestrian Push-Button** | 1 | Momentary push-button connected to `PD2 / INT0` with active-low pull-up configuration |
| **Light Sensor (LDR)** | 1 | Light-dependent resistor in a voltage divider with a 10 kΩ resistor connected to `PC2 / ADC2` |
| **Display** | 1 | 7-segment display for the pedestrian crossing countdown |
| **Current-Limiting Resistors** | As required | 220 Ω / 330 Ω resistors for LEDs and display segments |

---

## ⚙️ Pin-to-Port Mapping

| Signal / Peripheral | Arduino Pin | ATmega328P Port Pin | Function |
| :--- | :---: | :--- | :--- |
| **North/South Red** | D2 | `PD2` / `INT0` | Pedestrian interrupt / NS red |
| **North/South Yellow** | D3 | `PD3` | NS yellow traffic signal |
| **North/South Green** | D4 | `PD4` | NS green traffic signal |
| **East/West Red** | D5 | `PD5` | EW red traffic signal |
| **East/West Yellow** | D6 | `PD6` | EW yellow traffic signal |
| **East/West Green** | D7 | `PD7` | EW green traffic signal |
| **Display / Control Pins** | D8–D13, A0 | `PB0`–`PB5`, `PC0` | Segment control and system logic |
| **LDR Analog Input** | A2 | `PC2` / `ADC2` | Ambient light sampling |

---

## 🔄 Operating Modes

1. **Normal Traffic Mode:** Cycles through 8 deterministic states managing vehicle right-of-way across North/South and East/West directions with 2-second all-red clearance safety buffers.
2. **Pedestrian Crossing Mode:** Triggered by a button press; safely locks all vehicle lights to red and initiates a descending 5-to-0 countdown.
3. **Night Mode:** Activated when ambient light drops below the ADC threshold (`< 100` / `350`). All yellow LEDs flash simultaneously at 1 Hz until the sustained 5-second recovery threshold is met.

---

## 📂 Repository Structure

```text
├── Assembly/
│   └── ESD_Assignment/     # AVR Assembly source files (.asm) and Microchip Studio project
├── Schematics/             # Proteus Design Suite simulation files (.pdsprj)
├── Docs/                   # Full assignment report (IE3064_Assignment_Group_16.pdf)
└── README.md               # Project documentation
```

---

## 👥 Group 16 Members

- **IT24103485** — B.A. Sellapperuma
- **IT24610799** — M.L.P. Kalpage