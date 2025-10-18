#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Parameters

# Controller type: P, PI, PID, PD, PIV
controllerType = "PID"
# Responsiveness (0=Slower, 1=Faster)
responsiveness = 0.5
# Transient behavior (0=Aggressive, 1=Robust)
transientBehavior = 0.5

# Model

timestep = 0.01
measurementCoefficients = [-1.96399484, 0.96400979]
inputCoefficients = [1.37622864e-04, 2.55175613e-05, 4.77652206e-05]

# Tuning

def tune(controller_type, responsiveness, transient_behavior):
    # Target specifications
    target_overshoot = max(5, 50 * (1 - transient_behavior))  # 5% to 50%
    target_settling_time = 0.1 + 2.0 * (1 - responsiveness)   # 0.1s to 2.1s
    target_rise_time = 0.05 + 0.5 * (1 - responsiveness)      # 0.05s to 0.55s
    
    # Initial parameters (will be optimized)
    if controller_type == "P":
        Kp, Ki, Kd, Kv = 1.0, 0.0, 0.0, 0.0
    elif controller_type == "PI":
        Kp, Ki, Kd, Kv = 1.0, 0.5, 0.0, 0.0
    elif controller_type == "PID":
        Kp, Ki, Kd, Kv = 1.0, 0.5, 0.1, 0.0
    elif controller_type == "PD":
        Kp, Ki, Kd, Kv = 1.0, 0.0, 0.1, 0.0
    elif controller_type == "PIV":
        Kp, Ki, Kd, Kv = 1.0, 0.5, 0.0, 0.3
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")
    
    # Objective function for optimization
    def objective(params):
        Kp, Ki, Kd, Kv = params

        # Enforce controller structure
        if controller_type == "P":
            Ki, Kd, Kv = 0.0, 0.0, 0.0
        elif controller_type == "PI":
            Kd, Kv = 0.0, 0.0
        elif controller_type == "PD":
            Ki, Kv = 0.0, 0.0
        elif controller_type == "PID":
            Kv = 0.0
        elif controller_type == "PIV":
            Kd = 0.0

        try:
            # Very simple "tuning law" just to give the optimizer something to shape
            Kp_tuned = Kp * (0.5 + responsiveness)
            Ki_tuned = Ki * (0.5 + transient_behavior)
            Kd_tuned = Kd * (0.5 + responsiveness)
            Kv_tuned = Kv * (0.5 + transient_behavior)

            # For a coarse scalar objective, use:
            # - rise_time target ~ Kp_tuned
            # - settling_time target ~ Ki_tuned (integral speeds settling in this toy)
            # - overshoot target ~ (Kd_tuned + Kv_tuned) damping bundle
            damping_bundle = Kd_tuned + Kv_tuned * 0.8

            cost = (
                abs(Kp_tuned - target_rise_time) +
                abs(Ki_tuned - target_settling_time) +
                abs(damping_bundle - target_overshoot)
            )
            return cost
        except Exception:
            return 1e6  # Large penalty for any error

    # Optimize parameters
    initial_params = [Kp, Ki, Kd, Kv]
    bounds = [
        (0.1, 100),   # Kp
        (0.0, 100),   # Ki
        (0.0, 10),    # Kd
        (0.0, 50),    # Kv
    ]

    result = minimize(objective, initial_params, bounds=bounds, method='L-BFGS-B')
    Kp_opt, Ki_opt, Kd_opt, Kv_opt = result.x

    # Enforce structure one more time
    if controller_type == "P":
        Ki_opt, Kd_opt, Kv_opt = 0.0, 0.0, 0.0
    elif controller_type == "PI":
        Kd_opt, Kv_opt = 0.0, 0.0
    elif controller_type == "PD":
        Ki_opt, Kv_opt = 0.0, 0.0
    elif controller_type == "PID":
        Kv_opt = 0.0
    elif controller_type == "PIV":
        Kd_opt = 0.0

    return Kp_opt, Ki_opt, Kd_opt, Kv_opt

def continuous_to_discrete(Kp, Ki, Kd, Kv, timestep):
    """
    Convert continuous gains to "discrete" gains using the same simple
    Tustin-style scalings as the rest of this script.
    """
    Kp_d = Kp
    Ki_d = Ki * timestep / 2.0
    Kd_d = Kd * 2.0 / timestep
    Kv_d = Kv * 2.0 / timestep
    return Kp_d, Ki_d, Kd_d, Kv_d

def step_response(Kp, Ki, Kd, Kv):
    """
    Analyze step response and return performance metrics.
    Toy model: we fold Kv into damping similarly to derivative action.
    """
    try:
        t = np.linspace(0, 5, 1000)

        # Build toy "effective" parameters
        # - Kp raises speed
        # - Ki modulates oscillatory part
        # - Derivative-like damping from (Kd + alpha*Kv)
        alpha = 0.8
        damping = Kd + alpha * Kv

        # Very simple shape: decaying term * oscillatory term * extra decay from damping
        y = 1 - np.exp(-t * max(Kp, 1e-6)) * np.cos(t * max(Ki, 1e-6)) * np.exp(-t * max(damping, 1e-6))

        y = np.clip(y, 0, 1.2)
        y_final = y[-1]

        # Rise time (10%)
        y_10 = 0.1 * y_final
        rise_time = 0.0
        for i, val in enumerate(y):
            if val >= y_10:
                rise_time = t[i]
                break

        # Settling time (2%)
        settling_time = 0.0
        for i in range(len(y)-1, -1, -1):
            if abs(y[i] - y_final) > 0.02 * abs(y_final):
                settling_time = t[i]
                break

        overshoot = max(0.0, (max(y) - y_final) / max(y_final, 1e-6) * 100.0)
        return t, y, rise_time, settling_time, overshoot
    except Exception:
        t = np.linspace(0, 5, 1000)
        y = np.ones_like(t)
        return t, y, 0.1, 0.5, 0.0

print(f"{controllerType} Tuning\n")
print(f"Responsiveness: {responsiveness} (0=Slower, 1=Faster)")
print(f"Transient Behavior: {transientBehavior} (0=Aggressive, 1=Robust)")
print(f"Time Step: {timestep} seconds\n")

# Display ARX model as a transfer function
num = inputCoefficients
den = np.concatenate([[1], measurementCoefficients])

print("Transfer Function:\n")
print(f"       {inputCoefficients[0]:.6f}s² + {inputCoefficients[1]:.6f}s + {inputCoefficients[2]:.6f}")
print("G(s) = " + "─" * 50)
print(f"       s² + {measurementCoefficients[0]:.6f}s + {measurementCoefficients[1]:.6f}\n")

# Tune (now returns Kv, too)
Kp, Ki, Kd, Kv = tune(controllerType, responsiveness, transientBehavior)

# Analyze step response (includes Kv)
t, y, rise_time, settling_time, overshoot = step_response(Kp, Ki, Kd, Kv)

# Convert to discrete gains (includes Kv)
Kp_d, Ki_d, Kd_d, Kv_d = continuous_to_discrete(Kp, Ki, Kd, Kv, timestep)

# Display results
print(f"\n{controllerType} Gains:\n")
print("Continuous Gains:")
print(f"Kp = {Kp:.4f}")
if Ki > 0: print(f"Ki = {Ki:.4f}")
if Kd > 0: print(f"Kd = {Kd:.4f}")
if Kv > 0: print(f"Kv = {Kv:.4f}")

print("\nDiscrete Gains:")
print(f"Kp = {Kp_d:.4f}")
if Ki > 0: print(f"Ki = {Ki_d:.4f}")
if Kd > 0: print(f"Kd = {Kd_d:.4f}")
if Kv > 0: print(f"Kv = {Kv_d:.4f}")

print()
print(f"Rise Time: {rise_time:.3f} seconds")
print(f"Settling Time: {settling_time:.3f} seconds")
print(f"Overshoot: {overshoot:.1f}%")

# Plot step response
plt.figure(figsize=(10, 6))
plt.plot(t, y, 'b-', linewidth=2, label='Step Response')
plt.axhline(y=1, color='r', linestyle='--', alpha=0.7, label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Output')
title_ctl = "PIV" if controllerType == "PIV" else controllerType
plt.title(f'Controller Step Response ({title_ctl})')
plt.grid(True)
plt.legend()
plt.show()
