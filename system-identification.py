#!/usr/bin/env python3

import numpy as np
import pandas as pd
from scipy.linalg import lstsq
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

#
# Parameters
#

# Dataset to use
dataset = "rotation2.csv"
# Start time
start = 6
# End time
end = 25
# Gaussian filtering amount
smoothingFactor = 8
# Number of states
numberOfStates = 2
# Number of inputs
numberOfInputs = 2
# Delay time steps
delayTimeSteps = 1

#
# Load data
#

data = pd.read_csv(dataset)
time = data.iloc[:, 0].values
inputs = data.iloc[:, 1].values
measurements = data.iloc[:, 2].values

# Resample data to regular timestep
df = pd.DataFrame({
  'time': time,
  'inputs': inputs,
  'measurements': measurements
})

resampled = df.groupby('time').mean().reset_index()
time = resampled['time'].values
inputs = resampled['inputs'].values
measurements = resampled['measurements'].values

# Select time range
mask = (time >= start) & (time <= end)
time = time[mask]
inputs = inputs[mask]
measurements = measurements[mask]

# Apply Gaussian filter if smoothing factor > 0
if smoothingFactor > 0:
    measurements = gaussian_filter1d(measurements, sigma=smoothingFactor)
    print(f"Applied Gaussian filter with smoothing factor: {smoothingFactor}")

# Build regression matrix for ARX model
numberOfSamples = len(measurements)
maxDelay = max(numberOfStates, numberOfInputs + delayTimeSteps)
numberOfParameters = numberOfStates + numberOfInputs + 1
samples = np.zeros((numberOfSamples - maxDelay, numberOfParameters))

# State transition part (past outputs)
for i in range(numberOfStates):
    samples[:, i] = -measurements[maxDelay - 1 - i : numberOfSamples - 1 - i]

# Input-output mapping part (current and past inputs)
for i in range(numberOfInputs + 1):
    samples[:, numberOfStates + i] = inputs[maxDelay - delayTimeSteps - i : numberOfSamples - delayTimeSteps - i]

# Target vector
target = measurements[maxDelay:]

# Solve least squares: samples * weights = target
weights, _, _, _ = lstsq(samples, target)

# Identified parameters
stateTransitionParameters = weights[:numberOfStates]
inputParameters = weights[numberOfStates:]

# Validation
predictions = samples @ weights
meanSquaredError = np.mean((target - predictions)**2)
fit = 1 - np.sum((target - predictions)**2) / np.sum((target - np.mean(target))**2)

# Display results
print(f"State transition parameters: {stateTransitionParameters}")
print(f"Input parameters: {inputParameters}")
print(f"Mean Squared Error: {meanSquaredError * 100:.4f}%, Fit: {fit * 100:.2f}%")

# Display chart
plt.figure(figsize=(12, 8))

# Plot inputs over time
plt.subplot(2, 1, 1)
plt.plot(time, inputs, 'b-', label='Inputs', linewidth=1)
plt.xlabel('Time (s)')
plt.ylabel('Input')
plt.title('Inputs Over Time')
plt.grid(True)
plt.legend()

# Plot measurements and predictions over time
plt.subplot(2, 1, 2)
plt.plot(time, measurements, 'g-', label='Measurements', linewidth=1)
plt.plot(time[maxDelay:], predictions, 'r--', label='Predictions', linewidth=1)
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.title('Measurements vs Predictions Over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
