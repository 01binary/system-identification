#!/usr/bin/env python3

import numpy as np
import pandas as pd
from scipy.linalg import lstsq
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

# Parameters

dataset = "rotation2.csv"
startTime = 6
endTime = 25
gaussianFactor = 0
measurementTaps = 2
inputTaps = 2
delayTimeSteps = 1

# Load data

data = pd.read_csv(dataset)
time = data.iloc[:, 0].values
inputs = data.iloc[:, 1].values
measurements = data.iloc[:, 2].values

# Clean data

# Remove duplicate timestamps by averaging, then sort by time
df = pd.DataFrame({'time': time, 'inputs': inputs, 'measurements': measurements})
resampled = (
    df.groupby('time', as_index=False)
      .mean()
      .sort_values('time', kind='mergesort')
)
uniqueSamples = resampled['time'].to_numpy()

# Estimate timestep as median of successive diffs, less sensitive to jitter than mean
timeStep = np.median(np.diff(uniqueSamples))

# Build a regular grid without floating-point drift or extrapolation
t0 = uniqueSamples[0]
t1 = uniqueSamples[-1]
steps = int(round((t1 - t0) / timeStep)) + 1
regularTime = np.linspace(t0, t0 + (steps - 1) * timeStep, steps)

# Interpolate onto the regular grid
spanMask = (regularTime >= t0) & (regularTime <= t1)
regularTime = regularTime[spanMask]
inputs = np.interp(regularTime, uniqueSamples, resampled['inputs'].to_numpy())
measurements = np.interp(regularTime, uniqueSamples, resampled['measurements'].to_numpy())
time = regularTime

# Select analysis window after resampling
mask = (time >= startTime) & (time <= endTime)
time = time[mask]
inputs = inputs[mask]
measurements = measurements[mask]

# Apply Gaussian filter if smoothing factor > 0
if gaussianFactor > 0:
 measurements = gaussian_filter1d(measurements, sigma=gaussianFactor)

# Build regression matrix for ARX model
numberOfSamples = len(measurements)
maxDelay = max(measurementTaps, inputTaps + delayTimeSteps)
numberOfParameters = measurementTaps + inputTaps + 1
samples = np.zeros((numberOfSamples - maxDelay, numberOfParameters))

# State transition part (past outputs)
for i in range(measurementTaps):
    samples[:, i] = -measurements[maxDelay - 1 - i : numberOfSamples - 1 - i]

# Input-output mapping part (current and past inputs)
for i in range(inputTaps + 1):
    samples[:, measurementTaps + i] = inputs[maxDelay - delayTimeSteps - i : numberOfSamples - delayTimeSteps - i]

# Target vector
target = measurements[maxDelay:]

# Solve least squares: samples * weights = target
weights, _, _, _ = lstsq(samples, target)

# Identified parameters
measurementCoefficients = weights[:measurementTaps]
inputCoefficients = weights[measurementTaps:]

# Validation
predictions = samples @ weights
meanSquaredError = np.mean((target - predictions)**2)
fit = 1 - np.sum((target - predictions)**2) / np.sum((target - np.mean(target))**2)

# Display results
print(f"Measurement Coefficients: {measurementCoefficients}")
print(f"Input Coefficients: {inputCoefficients}")
print(f"Mean Squared Error: {meanSquaredError * 100:.4f}%, Fit: {fit * 100:.2f}%")

# Display chart
plt.figure(figsize=(12, 8))

# Plot inputs over time
plt.subplot(2, 1, 1)
plt.plot(time, inputs, 'b-', label='Inputs', linewidth=1)
plt.xlabel('Time (s)')
plt.ylabel('Input')
plt.title('Inputs')
plt.grid(True)
plt.legend()

# Plot measurements and predictions over time
plt.subplot(2, 1, 2)
plt.plot(time, measurements, 'g-', label='Measurements', linewidth=1)
plt.plot(time[maxDelay:], predictions, 'r--', label='Predictions', linewidth=1)
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.title('Measurements & Predictions')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
