# Design, Implementation, and Control of a 2-DOF Robotic Manipulator

This repository contains the documentation, design files, and control algorithms for a 2-degree-of-freedom (2-DOF) planar robotic manipulator. The project was developed as part of the **Analysis and Control of Robots (MT517)** course at the **National University of Engineering (UNI)**.

The robotic arm is designed to be integrated into a rover for search and rescue applications, capable of performing tasks such as sample collection and supply delivery.
<img width="1283" height="891" alt="image" src="https://github.com/user-attachments/assets/6cab6a90-f2d9-418e-bf80-56b61c8a53b9" />

# Design and Implementation
<img width="1111" height="805" alt="image" src="https://github.com/user-attachments/assets/c5f36822-d434-463a-8929-1381cbc83aaa" />

<img width="1214" height="683" alt="image" src="https://github.com/user-attachments/assets/dd33bbad-8b02-4fb4-b2cf-8d39ef91bf5b" />

## Repository Structure

The repository is currently organized into reports detailing the project's progression and a code folder for simulations:

* **`T1/`**: **Report 1 - Mechanical Design**
    * Selection of DC motors (Pololu 37D and 25D) with encoders.
    * Mechanical design using SolidWorks.
    * Kinematic analysis (Denavit-Hartenberg parameters).
* **`T2/`**: **Report 2 - Electronic Implementation & Assembly**
    * Electronic system design (Arduino Mega 2560, DRV8871 drivers).
    * PCB Shield design using Eagle.
    * Mechanical assembly and 3D printing of components.
    * Initial movement tests using PWM.
* **`T3/`**: **Report 3 - Dynamic Modeling & Control**
    * Dynamic modeling (Lagrange formulation, calculating inertia/centers of mass).
    * MATLAB/Simulink simulations.
    * Control system design (Decentralized control, trajectory planning).
* **`CODE/`**: Contains MATLAB scripts (`.m`) and Simulink models (`.slx`) for kinematic and dynamic simulations (e.g., `Demo_01.m`, `Control_optimo.m`).

## 🛠️ Hardware Specifications

* **Microcontroller:** Arduino Mega 2560 R3
* **Actuators:**
    * Joint 1: Pololu 37D Metal Gearmotor (70:1 ratio)
    * Joint 2: Pololu 25D Metal Gearmotor (78:1 ratio)
    * Gripper: Micro Servo
* **Drivers:** DRV8871 Motor Drivers
* **Power:** Li-ion 18650 Battery Pack (3S2P configuration)
* **Material:** 3D printed PLA parts and aluminum profiles.

## Usage

Currently, the repository serves as a documentation hub for the project's engineering process. You can view the PDF reports in their respective folders to understand the mathematical modeling and design choices.

To run the simulations:
1.  Navigate to the `CODE/` folder.
2.  Open the `.m` scripts in MATLAB or the `.slx` files in Simulink.
3.  Ensure you have the necessary toolboxes installed to run the dynamic simulations.

## Future Work

The repository is actively being updated. Upcoming uploads will include:
* **Arduino Firmware:** Complete C++ code for the Arduino Mega to control the physical robot, handle encoder interrupts, and execute trajectories.
* **Extended MATLAB/Simulink Control:** More advanced control strategies and real-time interface scripts.

## Authors

* **Aldave Torres, Carlos Felipe Sebastián**
* **Castro Suazo, Fidel Angel**
* **Contreras Avendaño, Juan Sebastián Francesco**
* **Llontop Herrera, Marco Leandro**

**Institution:** Universidad Nacional de Ingeniería (UNI) - Faculty of Mechanical Engineering
