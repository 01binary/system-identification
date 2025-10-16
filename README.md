# System Identification with ARX

Python scripts for identifying a system from data and tuning a PID controller.

Based on the first chapter of [System Identification: Theory for the User by Ljung](./SystemIdentificationLjung.pdf)

## Installation

Install dependencies:

```
./install.sh
```

## System Identification

Identify a system from a dataset:

```
./system-identification.py
```

Parameters at the top of the script:

|Parameter|Description|
|-|-|
|`dataset`|Which CSV dataset to use: `time`, `input`, `measurement`|
|`start`|Start time in seconds for samples to select|
|`end`|End time in seconds for samples to select|
|`smoothingFactor`|Gaussian filtering amount or `0` to disable|
|`numberOfStates`|How many states to estimate (i.e. 2 states = 2nd order model with position and velocity)
|`numberOfInputs`|How many inputs to estimate|
|`delayTimeSteps`|How many time steps are measurements delayed from inputs|

## PID Controller Tuning

Tune a PID controller:

```
./tuning.py
```

Parameters at the top of the script:

|Parameter|Description|
|-|-|
|`controllerType`|`P`, `PI`, `PID`, `PD`, `PIV`|
|`responsiveness`|Balance between speed and precision, (0 Slower...1 Faster)|
|`transientBehavior`|Balance between aggressiveness and neatness, (0 Aggressive...1 Robust)|