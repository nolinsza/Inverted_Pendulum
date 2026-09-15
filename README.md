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
  <img width="500" alt="State Space Equation" src="https://github.com/user-attachments/assets/caac17b2-3d18-4b89-acc1-c97115f8e267" />
</p>

<p align="center">
  <img width="500" alt="Angle vs Time Plot" src="https://github.com/user-attachments/assets/66b97d64-fe49-4afd-bde8-877b8b35282f" />
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
  <img width="300" alt="System Modelling Framework" src="https://github.com/user-attachments/assets/fd01825c-2ec5-41a3-96d3-a1b442dedfe1" />
</p>

<p align="center">
  <em>Figure 2. System Modelling Framework</em>
</p>

## Controller Design

The PD controller was tuned in simulation to produce underdamped, overdamped, and critically
damped responses before being implemented on the physical system.

<p align="center">
  <img width="600" alt="PD Control Block Diagram" src="https://github.com/user-attachments/assets/bffc5729-aa2e-4578-a7d1-b4e84103115a" />
</p>

<p align="center">
  <em>Figure 3. PD Control Block Diagram</em>
</p>



<p align="center">
  <img width="500" alt="Simulated Underdamped Response" src="https://github.com/user-attachments/assets/02acdf2a-b732-4d58-8108-9ca0366ef33f" />
</p>

<p align="center">
  <em>Figure 4.1. Simulated Underdamped Response</em>
</p>

<p align="center">
  <img width="500" alt="Simulated Overdamped Response" src="https://github.com/user-attachments/assets/7caf8ce2-2195-43a8-aabb-aec74ee9ccd8" />
</p>

<p align="center">
  <em>Figure 4.2. Simulated Overdamped Response</em>
</p>

<p align="center">
  <img width="500" alt="Simulated Critically Damped Response" src="https://github.com/user-attachments/assets/a89a61c4-a8c6-43be-bdf6-599b1d7f1438" />
</p>

<p align="center">
  <em>Figure 4.3. Simulated Critically Damped Response</em>
</p>

## Embedded Implementation

Figure 5 shows a general flowchart of the controller operating at a 100 Hz loop. The
initialization period includes arming the ESCs and configuring hardware timers to operate
the control loop and ADC oversampling.

<p align="center">
  <img width="450" alt="ESP32 Controller Flow Chart" src="https://github.com/user-attachments/assets/86577202-d418-4590-b9ad-8bab447cb22d" />
</p>

<p align="center">
  <em>Figure 5. ESP32 Controller Flow Chart</em>
</p>

### Sensor Processing

The rotary potentiometer provides an analog voltage proportional to the pendulum angle.
To reduce measurement noise, the ESP32 oversamples the ADC and averages 256 samples
before converting the measurement to angular position.

An exponential moving average filter with $\alpha = 0.15$ was applied to the measured
angle to reduce noise amplification during angular velocity calculation.

<p align="center">
  <img width="425" alt="Raw vs. Filtered Angle" src="https://github.com/user-attachments/assets/d38246ff-0c2b-439c-8158-7dd8b2b47cd4" />
</p>

<p align="center">
  <em>Figure 6.1. Raw vs. Filtered Angle</em>
</p>

<p align="center">
  <img width="425" alt="Raw vs. Filtered Angular Velocity" src="https://github.com/user-attachments/assets/235f5c9a-39a7-49ee-9440-0c87924f6dfc" />
</p>

<p align="center">
  <em>Figure 6.2. Raw vs. Filtered Angular Velocity</em>
</p>

<p align="center">
  <img width="425" alt="Raw vs. Filtered PWM Command" src="https://github.com/user-attachments/assets/1169797c-7033-46f6-bd5d-cf4e787e8b48" />
</p>

<p align="center">
  <em>Figure 6.3. Raw vs. Filtered PWM Command</em>
</p>

## Results

The completed system stabilizes indefinitely within approximately $\pm 3^\circ$ of the
upright position and recovers from small external disturbances. The system also
demonstrated underdamped, overdamped, and critically damped responses, with a measured
settling time of 6.15 seconds for the critically damped response.

<p align="center">
  <img width="975" alt="Launch Sequence" src="https://github.com/user-attachments/assets/de7e48dd-1956-4156-9f1c-f826ba2eeb1e" />
</p>

<p align="center">
  <em>Figure 7.1. Launch Sequence</em>
</p>

<p align="center">
  <img width="975" alt="Underdamped Response" src="https://github.com/user-attachments/assets/6d3b96d6-7c7a-4cba-86e7-8c27c493450a" />
</p>

<p align="center">
  <em>Figure 7.2. Underdamped Response</em>
</p>

<p align="center">
  <img width="975" alt="Overdamped Response" src="https://github.com/user-attachments/assets/1791e43f-bef3-4792-96e1-8bd87b683df1" />
</p>

<p align="center">
  <em>Figure 7.3. Overdamped Response</em>
</p>

<p align="center">
  <img width="975" alt="Critically Damped Response" src="https://github.com/user-attachments/assets/76b0d117-4352-43ac-8b98-86b2c3b31216" />
</p>

<p align="center">
  <em>Figure 7.4. Critically Damped Response</em>
</p>
