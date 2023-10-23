# FastBitShiftPIDController Library

## Description

### The Bit-Shifting Concept:
In the realm of binary arithmetic, bit shifting is akin to multiplication or division by powers of two. Specifically, when you shift a number to the left by `n` bits, you're effectively multiplying that number by \(2^n\). Conversely, shifting to the right by `n` bits divides the number by \(2^n\). This inherent efficiency is harnessed by the `FastBitShiftPIDController` to compute PID values swiftly.

However, there's a trade-off: using bit shifts for multiplication restricts our multipliers to powers of two. Thus, the gains achieved by our PID controller (Proportional, Integral, Derivative) are restricted to being powers of two. This makes it different from conventional PID controllers which can utilize floating-point multipliers.

### Parameters and Their Implications:

1. **`Kp_shift`, `Ki_shift`, `Kd_shift`:** These are bit shift values for Proportional, Integral, and Derivative terms respectively. A value of `n` corresponds to a gain of \(2^n\). A larger value amplifies the respective term. For instance, a `Kp_shift` of 4 would mean a proportional gain of 16 (or \(2^4\)).

2. **`scaleShift`:** Scales the error and integral terms to increase precision and to prevent overflow. The error and integral terms are multiplied by \(2^\text{scaleShift}\) before computation and divided by the same value afterward.

3. **`outputMin` and `outputMax`:** They represent the bounds for the output. They're crucial in applications like motor control to prevent input values that could be potentially harmful.

### Why and When to Use:
Opt for `FastBitShiftPIDController` when rapid computation trumps precision, as in real-time systems, embedded applications, or when computational resources are limited. However, when precise tuning is required, especially if the desired PID gains aren't close approximations to powers of two, conventional PID controllers might be more suitable.

## Examples

### 1. Basic Implementation:

```
#include "FastBitShiftPIDController.h"

int main() {
    FastBitShiftPIDController pid(3, 0, 0);
    int32_t setpoint = 37;
    int32_t currentTemperature = 35;

    int32_t output = pid.compute(setpoint, currentTemperature);
}
```

### 2. Comprehensive Implementation:

```
#include "FastBitShiftPIDController.h"

int main() {
    FastBitShiftPIDController pid(4, 2, 3, 8, -100, 100);
    int32_t setpoint = 100;
    int32_t currentAltitude = 95;

    while (true) {
        int32_t output = pid.compute(setpoint, currentAltitude);
        currentAltitude += (output / 10);
    }
}
```

### 3. Fine-Tuning Implementation:

```
#include "FastBitShiftPIDController.h"

int main() {
    FastBitShiftPIDController pid(5, 1, 6, 10, -200, 200);
    int32_t setpoint = 500;
    int32_t currentPosition = 450;

    while (true) {
        int32_t output = pid.compute(setpoint, currentPosition);
        currentPosition += (output / 5);
    }
}
```

