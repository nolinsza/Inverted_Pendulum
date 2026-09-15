# Inverted Pendulum Stabilization Project 

This system was designed in collaboration by Nolin Szafranski and Foster Gilmore

The project implements a real-time inverted pendulum stabilization system using an ESP32 microcontroller, rotary potentiometer feedback, and two brushless DC motors. A feedback controller continuously measures the pendulum angle and applies corrective thrust to maintain the pendulum in the upright position.

## Youtube Demonstration

1. Launch sequence — The pendulum is launched from approximately 60° and transitions into the critically damped operating condition
2. Underdamped response — The pendulum is reset to approximately 20° and allowed to demonstrate an underdamped response.
3. Overdamped response — The pendulum is reset to approximately 20° and demonstrates an overdamped response.
4. Critically damped response — The pendulum is returned to approximately 20° and demonstrates the final critically damped response.

##  Mathematical Model

The inverted pendulum was modeled using the free-body diagram shown below

<img width="316" height="406" alt="image" src="https://github.com/user-attachments/assets/3294de9d-ebbd-4c08-9bb9-558d4ab1607c" />
Figure 1. Inverted Pendulum Free Body Diagram 

### State-Space Model

Defining the system states as:

$$
x_1 = \theta
$$

$$
x_2 = \dot{\theta}
$$

with:

$$
\dot{x}_1 = x_2
$$

and

$$
\dot{x}_2 = \ddot{\theta}
$$

the linearized system can be represented in state-space form as:

$$
\dot{x}
=
\begin{bmatrix}
\dot{x}_1 \\
\dot{x}_2
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 \\
\frac{g}{l} & 0
\end{bmatrix}
x
+
\begin{bmatrix}
0 \\
-\frac{1}{ml}
\end{bmatrix}
u_f(t)
$$

where:

- $x_1$ is the pendulum angle
- $x_2$ is the angular velocity
- $u_f(t)$ is the applied control force
- $m$ is the pendulum mass
- $l$ is the pendulum length
- $g$ is the gravitational acceleration

This model was implemented in MATLAB to simulate the system response and determine appropriate controller gains.


