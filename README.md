# Inverted Pendulum Stabilization Project 

This system was designed in collaboration by Nolin Szafranski and Foster Gilmore

The project implements a real-time inverted pendulum stabilization system using an ESP32 microcontroller, rotary potentiometer feedback, and two brushless DC motors. A feedback controller continuously measures the pendulum angle and applies corrective thrust to maintain the pendulum in the upright position.

## YouTube Demonstration

1. Launch sequence — The pendulum is launched from approximately 60° and transitions into the critically damped operating condition
2. Underdamped response — The pendulum is reset to approximately 20° and allowed to demonstrate an underdamped response.
3. Overdamped response — The pendulum is reset to approximately 20° and demonstrates an overdamped response.
4. Critically damped response — The pendulum is returned to approximately 20° and demonstrates the final critically damped response.

##  Mathematical Model

The inverted pendulum was modeled and simulated in State-Space where uf(t) is the applied corrective force.


<img width="320.5" height="68.5" alt="image" src="https://github.com/user-attachments/assets/caac17b2-3d18-4b89-acc1-c97115f8e267" />

<img width="409.5" height="271.5" alt="image" src="https://github.com/user-attachments/assets/66b97d64-fe49-4afd-bde8-877b8b35282f" />

Figure 2. θ(t) from MATLAB Simulation 

## System Characterization

The mathematical model assumes a directly applied force, while the physical system is
controlled through motor PWM. Experimental testing was therefore performed to characterize
the relationship between PWM command and motor force.

A spring-based test setup was used to measure the force produced at different PWM commands.
The resulting data was used to develop a piecewise linear approximation of the PWM-to-force
relationship. The actuator response was also characterized with an estimated average time
constant of 0.1815 s.

<img width="521" height="939" alt="image" src="https://github.com/user-attachments/assets/fd01825c-2ec5-41a3-96d3-a1b442dedfe1" />
Figure 3. System Modelling Framework

## Controller Design

The PD controller was tuned in simulation to produce underdamped, overdamped, and critically damped responses before being implemented on the physical system.

<img width="975" height="346" alt="image" src="https://github.com/user-attachments/assets/bffc5729-aa2e-4578-a7d1-b4e84103115a" />
Figure 4. PD Control Block Diagram

<img width="622" height="423" alt="image" src="https://github.com/user-attachments/assets/02acdf2a-b732-4d58-8108-9ca0366ef33f" />
Figure 5.1 Simulated Underdamped Response 

<img width="576" height="391" alt="image" src="https://github.com/user-attachments/assets/7caf8ce2-2195-43a8-aabb-aec74ee9ccd8" />
Figure 5.2 Simulated Overdamped Response 

<img width="580" height="396" alt="image" src="https://github.com/user-attachments/assets/a89a61c4-a8c6-43be-bdf6-599b1d7f1438" />
Figure 5.3 Simulated Damped Response





