# Inverted Pendulum Stabilization Project 

This system was designed in collaboration by Nolin Szafranski and Foster Gilmore

The project implements a real-time inverted pendulum stabilization system using an ESP32 microcontroller, rotary potentiometer feedback, and two brushless DC motors. A feedback controller continuously measures the pendulum angle and applies corrective thrust to maintain the pendulum in the upright position.

## YouTube Demonstration

1. Launch sequence — The pendulum is launched from approximately 60° and transitions into the critically damped operating condition
2. Underdamped response — The pendulum is reset to approximately 20° and allowed to demonstrate an underdamped response.
3. Overdamped response — The pendulum is reset to approximately 20° and demonstrates an overdamped response.
4. Critically damped response — The pendulum is returned to approximately 20° and demonstrates the final critically damped response.

## Mathematical Model

The inverted pendulum was modeled and simulated in State-Space where $u_f(t)$ is the applied corrective force.

<p align="center">
  <img width="400" alt="State Space Equation" src="https://github.com/user-attachments/assets/caac17b2-3d18-4b89-acc1-c97115f8e267" />
</p>

<p align="center">
  <img width="410" alt="Angle vs Time Plot" src="https://github.com/user-attachments/assets/66b97d64-fe49-4afd-bde8-877b8b35282f" />
</p>

<p align="center">
  <em>Figure 1. θ(t) from MATLAB Simulation</em>
</p>

## System Characterization

The mathematical model assumes a directly applied force, while the physical system is
controlled through motor PWM. Experimental testing was therefore performed to characterize
the relationship between PWM command and motor force.

A spring-based test setup was used to measure the force produced at different PWM commands.
The resulting data was used to develop a piecewise linear approximation of the PWM-to-force
relationship. The actuator response was also characterized with an estimated average time
constant of 0.1815 s.

<p align="center">
  <img width="520" alt="System Modelling Framework" src="https://github.com/user-attachments/assets/fd01825c-2ec5-41a3-96d3-a1b442dedfe1" />
</p>

<p align="center">
  <em>Figure 3. System Modelling Framework</em>
</p>

## Controller Design

The PD controller was tuned in simulation to produce underdamped, overdamped, and critically
damped responses before being implemented on the physical system.

<p align="center">
  <img width="975" alt="PD Control Block Diagram" src="https://github.com/user-attachments/assets/bffc5729-aa2e-4578-a7d1-b4e84103115a" />
</p>

<p align="center">
  <em>Figure 4. PD Control Block Diagram</em>
</p>

<p align="center">
  <img width="620" alt="Simulated Underdamped Response" src="https://github.com/user-attachments/assets/02acdf2a-b732-4d58-8108-9ca0366ef33f" />
</p>

<p align="center">
  <em>Figure 5.1. Simulated Underdamped Response</em>
</p>

<p align="center">
  <img width="575" alt="Simulated Overdamped Response" src="https://github.com/user-attachments/assets/7caf8ce2-2195-43a8-aabb-aec74ee9ccd8" />
</p>

<p align="center">
  <em>Figure 5.2. Simulated Overdamped Response</em>
</p>

<p align="center">
  <img width="580" alt="Simulated Critically Damped Response" src="https://github.com/user-attachments/assets/a89a61c4-a8c6-43be-bdf6-599b1d7f1438" />
</p>

<p align="center">
  <em>Figure 5.3. Simulated Critically Damped Response</em>
</p>





