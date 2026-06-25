# Roboterarm HASE — Detailed Documentation

> **Project**: Open-source robot arm + control software  
> **Organization**: HASE (Humboldt Academy for Science and Engineering) at HGV Vaterstetten  
> **Repository**: https://github.com/HASE-HGV/Roboterarm-HASE  
> **License**: See `LICENSE`

---

## Table of Contents

1. [Team](#1-team)
2. [Project Overview](#2-project-overview)
3. [Hardware](#3-hardware)
   - 3.1 [Bill of Materials](#31-bill-of-materials)
   - 3.2 [Pin Mapping](#32-pin-mapping)
   - 3.3 [Wiring Diagram](#33-wiring-diagram)
   - 3.4 [A4988 Motor Driver Setup](#34-a4988-motor-driver-setup)
4. [Software Architecture](#4-software-architecture)
   - 4.1 [Repository Structure](#41-repository-structure)
   - 4.2 [rustctl — Main Control Program](#42-rustctl--main-control-program)
   - 4.3 [gpioTest — Hardware Test Utility](#43-gpiotest--hardware-test-utility)
   - 4.4 [Go Webserver (planned)](#44-go-webserver-planned)
5. [Inverse Kinematics](#5-inverse-kinematics)
   - 5.1 [Coordinate System](#51-coordinate-system)
   - 5.2 [Mathematical Model](#52-mathematical-model)
   - 5.3 [Step Conversion](#53-step-conversion)
6. [Motor Synchronization Algorithm](#6-motor-synchronization-algorithm)
7. [Installation & Usage](#7-installation--usage)
   - 7.1 [Prerequisites](#71-prerequisites)
   - 7.2 [Build & Run](#72-build--run)
   - 7.3 [CLI Parameters](#73-cli-parameters)
   - 7.4 [Interactive Mode](#74-interactive-mode)
8. [Known Issues & Limitations](#8-known-issues--limitations)
9. [Future Goals](#9-future-goals)

---

## 1. Team

| Member | Role |
|--------|------|
| **Patrick** | Software development, cable management, PCB design *(retired)* |
| **Luca** | Organisation, Rust development, 3D design, 3D printing, oscilloscope analysis |
| **Johannes** | Go webserver, Linux setup & guide |
| **Florian** | General support across tasks |
| **Julian** | Soldering, calculations & design review |

---

## 2. Project Overview

The Roboterarm HASE project aims to build a fully **open-source robotic arm** paired with **open-source control software** running on a Raspberry Pi. The arm has three active degrees of freedom:

- **Base rotation** — rotates the entire arm around the vertical axis (M3)
- **Shoulder joint** — first link of the planar arm (M1)
- **Elbow joint** — second link of the planar arm (M2)
- **Auxiliary axis** — fourth motor (M4) currently reserved/unused

The control software computes **3D inverse kinematics** to translate a target XYZ coordinate in millimeters into motor step counts, then drives four A4988 stepper motor drivers via the Raspberry Pi's GPIO pins.

The project is funded by sponsors and public grants and is part of the school's STEM program.

---

## 3. Hardware

### 3.1 Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| Stepper Motor (NEMA 17 or similar) | 4 | Connected via A4988 drivers |
| Raspberry Pi 4B+ | 1 | Main compute unit; runs rustctl |
| A4988 Stepper Motor Driver | 4 | Pololu-style breakout boards |
| Power Supply Unit (PSU) | 1 | Dual-rail: 5 V (logic) + 12 V (motors) |
| Arduino Mega | 1 | Used for oscilloscope/signal analysis |
| Rigol DS1052E Oscilloscope | 1 | Signal quality analysis |
| Aluminum profile (Alu-Profil) | — | Structural frame |
| Bambulab PETG-CF filament | — | Structural printed parts |
| Bambulab PLA-Matt filament | — | Non-structural printed parts |
| Reset switch | 1 | Shared RESET line across all 4 A4988 drivers |

### 3.2 Pin Mapping

Each A4988 driver requires a **STEP** signal and a **DIR** signal from the Raspberry Pi.

| Motor | Function | STEP (GPIO BCM) | DIR (GPIO BCM) |
|-------|----------|-----------------|----------------|
| M1 | Arm segment 1 (Shoulder) | GPIO 17 | GPIO 27 |
| M2 | Arm segment 2 (Elbow) | GPIO 22 | GPIO 23 |
| M3 | Base rotation | GPIO 24 | GPIO 25 |
| M4 | Auxiliary / unused | GPIO 5 | GPIO 6 |

The Raspberry Pi's 5 V pin powers the logic supply of all four A4988 boards. Motor power (12 V VMOT) is supplied by the PSU directly to each A4988 VMOT pin. All grounds (Pi GND, PSU GND, A4988 GND, A4988 VMOT GND) are connected together.

### 3.3 Wiring Diagram

```
PSU
├── 5V  ──► Raspberry Pi (power)
│         └► RESET SWITCH ──► RESET on A4988 (1–4)  [shared reset line]
├── 12V ──► VMOT on A4988 (1–4)
└── GND ──► Raspberry Pi GND
          ├► A4988 (1) GND + VMOT GND
          ├► A4988 (2) GND + VMOT GND
          ├► A4988 (3) GND + VMOT GND
          └► A4988 (4) GND + VMOT GND

Raspberry Pi GPIO
├── GPIO 17 ──► A4988 (1) STEP       A4988 (1) ──► M1: A1, A2, B1, B2
├── GPIO 27 ──► A4988 (1) DIR
├── GPIO 22 ──► A4988 (2) STEP       A4988 (2) ──► M2: A1, A2, B1, B2
├── GPIO 23 ──► A4988 (2) DIR
├── GPIO 24 ──► A4988 (3) STEP       A4988 (3) ──► M3: A1, A2, B1, B2
├── GPIO 25 ──► A4988 (3) DIR
├── GPIO  5 ──► A4988 (4) STEP       A4988 (4) ──► M4: A1, A2, B1, B2
└── GPIO  6 ──► A4988 (4) DIR
```

### 3.4 A4988 Motor Driver Setup

The A4988 is a microstepping driver with built-in translator. Key points:

- **STEP**: Each rising edge advances the motor by one microstep.
- **DIR**: Sets the rotation direction (HIGH = one direction, LOW = the other).
- **RESET / SLEEP**: Both pins must be pulled HIGH for the driver to operate. The shared reset switch allows a hardware reset of all four drivers simultaneously.
- **Microstepping**: Set via MS1/MS2/MS3 pins. Common values: 1 (full), 2 (half), 4, 8, 16 steps per full step.
- **Current limit**: Set by a potentiometer on the A4988 board. Must be tuned to match the motor's rated current.
- **VMOT capacitor**: A 100 µF capacitor should be placed close to each driver's VMOT/GND pins to suppress voltage spikes.

---

## 4. Software Architecture

### 4.1 Repository Structure

```
Roboterarm-HASE/
├── rustctl/                    # Main control program (Rust)
│   ├── Cargo.toml
│   └── src/main.rs
├── gpioTest/                   # Early GPIO test / motor test utility (Rust)
│   ├── Cargo.toml
│   └── src/main.rs
├── VI21-Anlagen/               # Project documentation assets
│   ├── Assembly 1 Drawing 1.svg
│   ├── Render1.png
│   ├── 50kPulseLongTimeAnalysis.png
│   ├── DS1ET210300369_0.jpg     # Oscilloscope screenshot
│   └── 0001-0520.avi            # Demo video
├── renderer*.blend             # Blender 3D model files (v1–v4)
├── inverseKinematics.ods       # LibreOffice spreadsheet: IK prototype/calculations
├── pins.md                     # Cable/pin mapping reference
├── A4988.png                   # A4988 driver pinout reference image
├── pinout.xyz.png              # Raspberry Pi GPIO pinout reference
├── compile.cmd                 # Cross-compile script for Go webserver (ARM)
├── arduino-oscilloscope/       # Git submodule: oscilloscope via Arduino
├── .github/
│   ├── workflows/python-package.yml
│   └── ISSUE_TEMPLATE/bug_report.md
└── README.md
```

### 4.2 rustctl — Main Control Program

**Language**: Rust (edition 2024)  
**Dependencies**:
- [`rppal`](https://crates.io/crates/rppal) `0.22.1` — Raspberry Pi Peripheral Access Library (GPIO)
- [`ctrlc`](https://crates.io/crates/ctrlc) `3.5.2` — Graceful Ctrl+C / SIGTERM handling

**What it does**:

1. **Input parsing** — Accepts either CLI arguments or drops into an interactive prompt mode (if no args are provided).
2. **Inverse kinematics** — Computes `theta_base`, `theta1`, `theta2` from a 3D target coordinate `(x, y, z)` and two arm-link lengths `(l1, l2)`.
3. **Step calculation** — Converts angles to motor step counts using the gear ratio (16:1) and microstep setting.
4. **GPIO setup** — Initializes 8 GPIO pins (4× STEP + 4× DIR) as outputs via `rppal`.
5. **Signal handler** — Registers a Ctrl+C handler that stops all motors safely (resets all pins LOW).
6. **Synchronized motion** — Uses a Bresenham-like accumulator algorithm to distribute steps across three axes in one unified timing loop.
7. **Cleanup** — Resets all GPIO pins after movement completes.

**Motor mapping in code**:

```
m1 (GPIO 17/27) → Shoulder (Arm segment 1)   → steps1     → theta1
m2 (GPIO 22/23) → Elbow    (Arm segment 2)   → steps2     → theta2
m3 (GPIO 24/25) → Base rotation               → steps_base → theta_base
m4 (GPIO  5/ 6) → Auxiliary                  → (unused)
```

### 4.3 gpioTest — Hardware Test Utility

**Language**: Rust (edition 2024)  
**Dependencies**: `rppal` only (no `ctrlc`)

This is an earlier, simpler test program used to verify that stepper motors respond to GPIO pulses before the IK logic was implemented. It:

- Asks the user how many motors to run (1–4).
- Asks for a delay in microseconds.
- Pulses 1–4 STEP pins simultaneously in an infinite loop.
- Does **not** set direction (no DIR pin control).
- Does **not** implement graceful shutdown — requires a hardware reset or `kill` to stop.

> Note: This program uses only STEP pins (GPIO 17, 22, 24, 5). It shares STEP pins with `rustctl` but ignores the DIR pins.

### 4.4 Go Webserver (planned)

`compile.cmd` contains a cross-compile command for Go targeting Linux ARM (ARMv6):

```bat
set GOOS=linux
set GOARCH=arm
set ARM=6
go build -o core
```

This indicates a Go-based webserver was planned (led by Johannes) to provide a browser-based UI for controlling the arm. **No Go source files are currently present in the repository.** This is a significant gap — see [Future Goals](#9-future-goals).

---

## 5. Inverse Kinematics

### 5.1 Coordinate System

```
         Z (up)
         │
         │   / Y (forward / base rotation plane)
         │  /
         │ /
         └──────── X (right)
         Origin = arm base
```

The arm is modeled as two rigid links in a vertical plane, with the entire plane rotating around the Z-axis (base rotation).

### 5.2 Mathematical Model

**Function**: `ik_angles_3d_deg(x, y, z, l1, l2) → (theta_base, theta1, theta2, z_eff)`

**Step 1 — Base rotation**:
```
theta_base = atan2(y, x)
```
Rotates the arm's plane of motion to face the target point.

**Step 2 — Project to 2D plane**:
```
r = sqrt(x² + y²)          // horizontal reach (ignoring z)
r_space = sqrt(r² + z²)    // total distance from origin to target
```

**Step 3 — Reachability check**:
```
if r_space > l1 + l2 → "Out of workspace" (error)
```

**Step 4 — Elbow angle (theta2)** using the law of cosines:
```
cos(theta2) = (r_space² − l1² − l2²) / (2 · l1 · l2)
theta2 = acos(cos(theta2))    // clamped to [-1, 1] to prevent NaN
```

**Step 5 — Shoulder angle (theta1)**:
```
alpha  = atan2(z, r)
theta1 = alpha − atan2(l2 · sin(theta2), l1 + l2 · cos(theta2))
```

**Step 6 — Effective Z** (verification / feedback):
```
z_eff = l1 · sin(theta1) + l2 · sin(theta1 + theta2)
```

The angles are returned in degrees for display and then converted to motor steps.

### 5.3 Step Conversion

**Function**: `deg_to_steps(angle_deg, steps_per_rev, microstep) → i64`

```
steps_per_degree = (steps_per_rev × microstep) / 360
steps = angle_deg × steps_per_degree × GEAR_RATIO
```

- `GEAR_RATIO = 16.0` (hardcoded — 16:1 reduction gearbox on each motor)
- Typical values: `steps_per_rev = 200` (1.8°/step motor), `microstep = 16`
- Sign of the result determines direction (positive → one direction, negative → the other)

---

## 6. Motor Synchronization Algorithm

The main movement loop uses a **Bresenham-like integer accumulator** to synchronize three motors moving different numbers of steps, so they all start and finish simultaneously:

```
max_steps = max(|steps_base|, |steps1|, |steps2|)

for each tick in 0..max_steps:
    accum_b += |steps_base|;  if accum_b >= max_steps → pulse motor_base, accum_b -= max_steps
    accum1  += |steps1|;      if accum1  >= max_steps → pulse motor1,    accum1  -= max_steps
    accum2  += |steps2|;      if accum2  >= max_steps → pulse motor2,    accum2  -= max_steps

    Set all pulsed STEP pins HIGH
    sleep(pulse_t_us)
    Set all pulsed STEP pins LOW
    sleep(pulse_t_us)
    sleep(overhead)           // remainder of the target step period
```

This ensures that every motor completes its required steps in exactly `max_steps` ticks, distributing the steps evenly throughout the motion.

**Timing parameters**:
- `total_time`: target period per tick (µs) — controls overall movement speed
- `pulse_t_us`: HIGH/LOW pulse width (µs) — must satisfy A4988 minimum pulse spec (≥1 µs)
- `minimum_required_time = 2 × pulse_t_us + 83` — 83 µs accounts for code overhead (mutex locks, etc.)

---

## 7. Installation & Usage

### 7.1 Prerequisites

| Requirement | Notes |
|-------------|-------|
| Raspberry Pi 4B (or any Pi with GPIO) | Running Raspberry Pi OS (64-bit recommended) |
| Rust toolchain | Install via `curl https://sh.rustup.rs -sSf \| sh` |
| GPIO access | Run as root or add user to `gpio` group |
| Physical hardware wired | See Section 3 |

### 7.2 Build & Run

```bash
git clone https://github.com/HASE-HGV/Roboterarm-HASE.git
cd Roboterarm-HASE/rustctl
cargo build --release
sudo ./target/release/rustctl
```

> `sudo` is required for GPIO access unless udev rules are configured.

### 7.3 CLI Parameters

```
rustctl <total_time_µs> <pulse_t_µs> <x_mm> <y_mm> <z_mm> <l1_mm> <l2_mm> <steps_per_rev> <microstep> <ccw_positive>
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `total_time_µs` | u64 | Target duration of each step cycle in microseconds |
| `pulse_t_µs` | u64 | STEP pulse HIGH duration in microseconds |
| `x_mm` | f64 | Target X coordinate in millimeters |
| `y_mm` | f64 | Target Y coordinate in millimeters (drives base rotation) |
| `z_mm` | f64 | Target Z coordinate in millimeters |
| `l1_mm` | f64 | Length of arm segment 1 (shoulder to elbow) in mm |
| `l2_mm` | f64 | Length of arm segment 2 (elbow to TCP) in mm |
| `steps_per_rev` | u64 | Motor full-step count per revolution (typically 200) |
| `microstep` | u64 | Microstepping divisor set on A4988 (1, 2, 4, 8, or 16) |
| `ccw_positive` | 0/1 | 1 = counterclockwise is the positive direction, 0 = clockwise |

**Example** (move to X=100mm, Y=0mm, Z=50mm with 200mm arm segments):
```bash
sudo ./target/release/rustctl 1000 200 100 0 50 200 200 200 16 1
```

### 7.4 Interactive Mode

Run without arguments to be prompted for each parameter:
```bash
sudo ./target/release/rustctl
```
The program will ask for each value and validate input, retrying on invalid entries.

---

## 8. Known Issues & Limitations

| # | Issue | Severity | Notes |
|---|-------|----------|-------|
| 1 | **No homing / calibration** | High | The arm has no limit switches or encoders. Absolute position is unknown at startup; accumulated error increases over time. |
| 2 | **No position tracking** | High | Steps are sent but not confirmed. There is no closed-loop feedback to detect missed steps or stalls. |
| 3 | **Motor M4 is unused** | Medium | GPIO 5/6 are configured but never commanded. M4 has no assigned IK axis. |
| 4 | **Go webserver missing** | Medium | `compile.cmd` and CONTRIBUTING.md reference a Go-based webserver component, but no Go source files exist in the repo. |
| 5 | **No Python code despite CI** | Low | The GitHub Actions workflow runs Python linting/tests but there is no Python code in the repository. |
| 6 | **CONTRIBUTING.md references missing folders** | Low | Mentions `hw-controller` and `hw-sim` directories that do not exist. |
| 7 | **Gear ratio hardcoded** | Low | `GEAR_RATIO = 16.0` in `deg_to_steps()` is hardcoded; should be a CLI/config parameter. |
| 8 | **No workspace boundary enforcement** | Medium | Only reachability (r_space ≤ l1+l2) is checked; no joint-angle limits, collision zones, or minimum reach (r_space < |l1-l2|) check. |
| 9 | **No acceleration/deceleration** | Medium | Steps are executed at constant speed; sudden starts/stops can cause missed steps at high speeds. |
| 10 | **gpioTest has no graceful shutdown** | Low | Requires hardware kill; pins may be left HIGH, keeping a motor coil energized and hot. |

---

## 9. Future Goals

### Short-Term (Next Semester)

- **Homing sequence**  
  Add limit switches (microswitches or hall-effect sensors) to all three active axes. Implement a homing routine that moves each axis to its known zero position at startup to establish absolute coordinates.

- **Go webserver implementation**  
  Implement the planned Go webserver (`hw-controller`) that exposes a REST API for sending target coordinates. A simple browser UI (HTML/JS) would allow controlling the arm from any device on the same network without needing SSH access.

- **Motor 4 end-effector**  
  Assign M4 to a gripper or wrist rotation. Extend the IK model to include a fourth degree of freedom and add grip-open/grip-close commands.

- **Configurable gear ratio**  
  Move the hardcoded `GEAR_RATIO = 16.0` constant to a CLI parameter or a `config.toml` file so the software works with different hardware configurations without recompiling.

### Medium-Term

- **Acceleration / deceleration profiles (S-curve or trapezoidal)**  
  Implement a ramping algorithm so the motors smoothly accelerate at the start and decelerate at the end of each move. This dramatically reduces missed steps at higher speeds.

- **Closed-loop step verification (optional)**  
  Add quadrature encoders or magnetic rotary encoders to each axis. Feed encoder data back to the Raspberry Pi via SPI or I²C to detect missed steps and implement basic position correction.

- **Joint angle limits**  
  Enforce per-joint min/max angle boundaries in software to prevent the arm from hitting its own structure. Also add a minimum-reach check (`r_space > |l1 - l2|`) to prevent the fully extended/retracted singularity.

- **Path planning**  
  Implement linear interpolation in Cartesian space (straight-line TCP paths) by splitting a long move into many small IK steps, rather than the current single-move IK call. This prevents the arm from sweeping unexpected arcs between positions.

- **GPIO simulator / software-in-the-loop testing**  
  Create the `hw-sim` component referenced in CONTRIBUTING.md: a mock GPIO layer that intercepts `rppal` calls and logs or visualizes step pulses without needing real hardware. This enables development and testing on a standard laptop.

### Long-Term

- **ROS 2 integration**  
  Wrap the Rust control layer in a ROS 2 node, exposing the arm as a standard `MoveIt!`-compatible manipulator. This would enable trajectory planning, obstacle avoidance, and integration with vision systems.

- **Computer vision pick-and-place**  
  Integrate a camera (e.g., Raspberry Pi Camera Module 3) and an object detection model to autonomously identify and pick up objects placed in front of the arm.

- **Web-based 3D simulation**  
  Use the existing Blender models to export a URDF or GLTF model and render a live 3D preview of the arm's pose in the browser alongside the webserver UI.

- **Multi-arm coordination**  
  Extend the software architecture to coordinate two or more arms on the same network, enabling collaborative assembly tasks.

- **PCB design for motor controller**  
  Design a custom PCB that integrates the four A4988 drivers, the reset circuit, and a Raspberry Pi HAT connector into a single board, replacing the current breadboard/jumper-wire setup with a production-quality solution.

---

*Documentation written June 2026. For questions or contributions, open an issue at https://github.com/HASE-HGV/Roboterarm-HASE/issues*
