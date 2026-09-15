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

### State-Space Model
<img width="641" height="137" alt="image" src="https://github.com/user-attachments/assets/caac17b2-3d18-4b89-acc1-c97115f8e267" />

<img width="819" height="543" alt="image" src="https://github.com/user-attachments/assets/66b97d64-fe49-4afd-bde8-877b8b35282f" />
Figure 2. θ(t) from MATLAB Simulation 




