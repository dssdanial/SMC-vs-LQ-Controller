# SMC-vs-LQR-Controller
Analysis and comparison of two controllers on a linearized system, Sliding-Mode vs Linear Quadratic Controller. In this case study, we focused on comparing two control approaches: Linear Quadratic Regulator (LQR) and Sliding Mode Control (SMC) on a linearized model of the inverted pendulum system. Applying LQR and SMC to the linearized model allows for a clearer analysis of their respective control laws, stability margins, and robustness against disturbances while avoiding the computational complexity of a fully nonlinear model.
![Inverted Pendulum_v3](https://github.com/user-attachments/assets/517605a6-f830-4f11-b6b8-6e3289a749b0)

### Motivations:
- Understand the fundamental behavior of LQR and SMC controllers in stabilizing an inherently unstable system.
- Evaluate the effect of dead zones on the performance of both controllers, particularly in terms of settling time, stability, and steady-state error.
- Establish a foundation for future work on nonlinear control by first analyzing the controllers on a simplified linear model.


### 1. Linear Quadratic Regulator (LQR)

The **Linear Quadratic Regulator (LQR)** is a state-feedback controller that minimizes a quadratic cost function. It’s widely used for controlling linear systems due to its ability to balance control performance and energy efficiency. The LQR controller minimizes the cost function:

$J = \int_0^{\infty} (x^T Q x + u^T R u) \, dt$

where:
- \( x \) is the state vector,
- \( u \) is the control input,
- \( Q \) and \( R \) are weighting matrices that determine the relative importance of state error and control effort.

The optimal feedback gain \( K \) for LQR is given by:

$K = R^{-1} B^T P$

where \( P \) is the solution to the continuous-time Algebraic Riccati Equation:

$A^T P + PA - PBR^{-1}B^T P + Q = 0$

This feedback gain \( K \) provides control action as \( u = -Kx \).



### 2. Sliding Mode Control (SMC)

**Sliding Mode Control (SMC)** 
SMC is a robust, nonlinear control technique that drives system states to a predefined sliding surface and then keeps them there, forcing the system to behave in a desired way. The sliding surface \( s(x) \) is defined such that when \( s(x) = 0 \), the system exhibits desired dynamics.

The control law in SMC has two components:
- **Equivalent control** $\( u_{\text{eq}} \)$: Drives the system to the sliding surface.
- **Switching control** $\( u_{\text{sw}} \)$: Maintains the system on the sliding surface once it reaches it.

The SMC control law is typically of the form:

$u = u_{\text{eq}} + u_{\text{sw}} = (S B)^{-1} S A x + k \cdot \text{sign}(s(x))$

where \( k \) is a tuning parameter, and \( S \) is the sliding surface matrix. The switching control introduces high-frequency switching, which is beneficial for robustness but may lead to chattering (oscillations near the surface).


### 3. Nonlinear Equations of Motion

The inverted pendulum system consists of a cart with a pendulum attached to it. Let:
- \( x \) represents the cart's horizontal position,
- \( \theta \) be the pendulum angle (with \( \theta = 0 \) indicating the upright position).

The equations of motion for this system are:

$M \ddot{x} + m \ell \ddot{\theta} \cos \theta - m \ell \dot{\theta}^2 \sin \theta = u$
$I \ddot{\theta} + m g \ell \sin \theta = -m \ell \ddot{x} \cos \theta$

where:
- \( M \) is the cart mass,
- \( m \) is the pendulum mass,
- \( $\ell$ \) is the length from the pivot to the center of mass of the pendulum,
- \( g \) is the acceleration due to gravity,
- \( u \) is the control input (force applied to the cart).

### 4. Linearization and State-Space Representation

To simplify the control design, we linearize the system around the upright position (\( \theta = 0 \), \( \dot{\theta} = 0 \)). The state-space representation of the linearized system is:

$\dot{x} = A x + B u$

where $\( x = [x, \dot{x}, \theta, \dot{\theta}]^T \)$. The matrices \( A \) and \( B \) are derived by taking the Jacobian of the nonlinear system around the equilibrium point.

The linearized state-space matrices are:

$$\
A = 
\begin{bmatrix} 
  0 & 1 & 0 & 0 \\ 
  0 & 0 & \frac{m g}{M} & 0 \\ 
  0 & 0 & 0 & 1 \\ 
  0 & 0 & \frac{g (M + m)}{l M} & 0 
\end{bmatrix}
\$$

$$\
B = 
\begin{bmatrix} 
  0 \\ 
  \frac{1}{M} \\ 
  0 \\ 
  \frac{1}{l M} 
\end{bmatrix}
\$$



### 5. Simulation Setup

The simulation was run for two scenarios:
1. **Without Dead Zone**: Control input operates without restrictions.
2. **With Dead Zone**: Control input has a dead zone threshold to reduce sensitivity to small errors.

For each scenario, both the **LQR** and **SMC** controllers were tested, and the responses were recorded for:
- Control input \( u \)
- Horizontal position \( x \)
- Pendulum angle \( \theta \)


### 6. Control Input (\( u \))

The control input shows that:
- **Without Dead Zone**: Both controllers respond aggressively at first to stabilize the system. SMC has a higher peak due to its robustness properties.
- **With Dead Zone**: Both controllers exhibit minor oscillations, and the system takes slightly longer to stabilize. Dead zones reduce control activity, smoothing out minor fluctuations.

#### Horizontal Position (\( x \))

- **Without Dead Zone**: Both controllers quickly bring the cart to the desired position, with LQR exhibiting smoother control, while SMC overshoots slightly but stabilizes faster.
- **With Dead Zone**: Both controllers experience oscillations due to delayed corrections within the dead zone. This results in a slightly longer settling time and oscillatory response.

#### Pendulum Angle (\( \theta \))

- **Without Dead Zone**: Both controllers achieve good stabilization. SMC stabilizes slightly faster but exhibits initial overshoot.
- **With Dead Zone**: The dead zone introduces oscillations in the pendulum angle as well, with both controllers requiring more time to achieve precise stabilization.

### 7. Technical Analysis

1. **Impact of Dead Zone**:
   - The dead zone reduces control responsiveness by ignoring small input values, resulting in less precise control and increased oscillations in position and angle. However, it improves smoothness, reducing the control effort and prolonging actuator lifespan.

2. **Controller Performance (LQR vs. SMC)**:
   - **LQR**: Provides smoother control actions, especially when a dead zone is present, due to its energy-minimizing properties.
   - **SMC**: Reacts more aggressively and is more robust to model uncertainties, but tends to oscillate more with dead zones due to its switching behavior.

3. **Recommended Controller**:
   - For systems requiring precision and minimal oscillations, **LQR** is preferable, especially in the presence of a dead zone.
   - For robustness against disturbances and model uncertainties, **SMC** offers better control but requires additional tuning to minimize oscillations.


## 8. Conclusion

This report demonstrates the behavior of **LQR** and **SMC** controllers on a linearized inverted pendulum model. The **LQR controller** provides smoother, energy-efficient control, while **SMC** offers robust performance with aggressive corrections. Dead zones improve smoothness but introduce oscillations, highlighting the trade-offs in controller design.

Further work can focus on extending these controllers to the **nonlinear system model** to evaluate robustness and control accuracy in a more realistic setting.


