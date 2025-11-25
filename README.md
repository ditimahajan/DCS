🚀 Satellite Altitude Control Using Discrete-Time LQR (LQI)

A MATLAB Implementation of Digital Station-Keeping Using Clohessy–Wiltshire Dynamics


---

📌 Project Overview

This project implements a discrete-time optimal controller (LQR with integral action, LQI) for satellite altitude control in Low Earth Orbit (LEO).

Using the Clohessy–Wiltshire (Hill) equations, we model small radial deviations from a reference circular orbit and design a digital controller that stabilizes altitude, rejects disturbances (e.g., drag), and minimizes fuel consumption (Δv).

The complete system is implemented in MATLAB, including:

Continuous-time orbital model

Zero-Order Hold (ZOH) discretization

Augmented LQI controller design

Simulation with disturbances and thruster saturation

Plots of radial error, control effort, Δv

Results saved to output folder


This project is suitable for academic coursework, research demonstrations, and introductory spacecraft control studies.


---

✨ Features

Accurate single-axis radial dynamics using CW equations

Automatic discretization using MATLAB’s c2d(...,'zoh')

LQI (LQR + integral action) for zero steady-state error

Thruster saturation limits

Integrator anti-windup (freeze logic)

Drag disturbance simulation

Fuel usage estimation via cumulative Δv

Clean plots saved automatically

Fully documented and easy to modify



---

📐 Mathematical Model

Continuous-Time Dynamics (CW radial equation)

\ddot{x} = 3n^2 x + u + d(t)

State-space form:

\dot{x} =
\begin{bmatrix}
0 & 1\\
3n^2 & 0
\end{bmatrix} x + 
\begin{bmatrix}
0\\ 1
\end{bmatrix} u

Discrete-Time (ZOH)

Using sample period :

A_d = e^{AT_s}, \qquad 
B_d = \int_0^{T_s} e^{A\tau}B\, d\tau

MATLAB:

sysd = c2d(ss(A,B,C,0), Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;


---

🎯 LQI Controller

To eliminate steady-state error, an integrator state is added:

z[k+1] = z[k] + T_s (r - y[k])

Augmented matrices:

A_a = \begin{bmatrix} A_d & 0 \\ -T_s C & 1 \end{bmatrix}, \quad
B_a = \begin{bmatrix} B_d \\ 0 \end{bmatrix}

LQR cost:

J = \sum (x_a^T Q x_a + u^T R u)

Control law:

u[k] = -K_x x[k] - K_i z[k]


---

🧠 MATLAB Script Included

The included script performs:

Model discretization

LQI gain computation using dlqr

Disturbance injection

Saturation and anti-windup

Simulation over 1 hour

Generation of 4 plots

Saving all figures + workspace


Run it directly:

run('satellite_lqi_control.m');


---

📊 Generated Plots

The script outputs:

1. Radial Error Response


2. Control Acceleration


3. Cumulative Δv (Fuel Usage)
