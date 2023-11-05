# FastBitShiftPIDController Library

## Description

The FastBitShiftPIDController library provides a high-performance, integer-based PID (Proportional-Integral-Derivative) controller implementation optimized for embedded systems with limited processing power. Unlike traditional floating-point PID controllers, this library leverages bit shifting for arithmetic operations, allowing for faster computation without the need for costly floating-point arithmetic. This makes the FastBitShiftPIDController particularly suitable for microcontroller applications where execution speed and resource efficiency are critical.

## Features

- **Integer-based calculations**: Avoids floating-point operations, enhancing speed and efficiency.
- **Configurable bit-shift scaling**: Fine-tune PID constants using bit shift enumeration for optimal performance.
- **Integral windup prevention**: Bounds integral term to prevent control saturation.
- **Optional integral and derivative term disable**: Allows use of PI or PD control strategies when necessary.
- **Output clamping**: Ensures the output stays within user-defined minimum and maximum bounds.

## Why Use FastBitShiftPIDController?

You would use the FastBitShiftPIDController library if you need a PID control solution that is:

- **Fast**: Designed with performance in mind, ideal for time-critical applications.
- **Resource-friendly**: Minimizes memory and CPU usage, perfect for constrained embedded systems.
- **Versatile**: Adjustable to a wide range of control tasks with simple configuration.
- **Deterministic**: Offers predictable execution time, essential for real-time systems.

## Methods

### Constructor

```
FastBitShiftPIDController(BitShift Kp_shift, BitShift Ki_shift, BitShift Kd_shift, BitShift scaleShift = BitShift::FAST_1024, int32_t outputMin = -255, int32_t outputMax = 255);
```

- Initializes the PID controller with bit shift values for Kp, Ki, and Kd.
- Allows setting of scaling for the PID computation.
- Accepts optional parameters for output range (min/max).

### Setters

```
void setKpShift(BitShift newKpShift);
void setKiShift(BitShift newKiShift);
void setKdShift(BitShift newKdShift);
void setPIDShifts(BitShift newKpShift, BitShift newKiShift, BitShift newKdShift);
void setIntegralDisabled(bool disable);
void setDerivativeDisabled(bool disable);
```

- These methods allow dynamic adjustment of PID parameters and control features.

### Compute Method

```
int32_t compute(int32_t setpoint, int32_t input);
```

- Computes the PID output given a setpoint and input value.
- Manages integral windup and respects output limits.

## Enum `BitShift`

The `BitShift` enum defines constants used to configure the bit shifting for PID term scaling:

```
enum class BitShift : uint8_t {
    FAST_1,    // No scaling, multiply/divide by 1
    FAST_2,    // Multiply/divide by 2
    // ... and so on
    FAST_4096  // Multiply/divide by 4096
};
```

The use of `BitShift` allows for quick multiplication or division by powers of two, substituting expensive arithmetic operations with efficient bitwise shifts.

## Example Usage

```
#include "FastBitShiftPIDController.h"

FastBitShiftPIDController pid(BitShift::FAST_4, BitShift::FAST_8, BitShift::FAST_16);

void setup() {
    // Configuration, if needed
    pid.setKpShift(BitShift::FAST_2);
    pid.setKiShift(BitShift::FAST_4);
    pid.setKdShift(BitShift::FAST_8);
    
    pid.setIntegralDisabled(false); // Enable integral term
    pid.setDerivativeDisabled(true); // Disable derivative term for PI Controller
}

void loop() {
    // Read process variable, e.g., sensor input
    int32_t input = readProcessInput();
    
    // Desired setpoint
    int32_t setpoint = 1000;
    
    // Compute PID output
    int32_t output = pid.compute(setpoint, input);
    
    // Apply PID output to your system, e.g., actuator
    applyControl(output);
}
```

In this example, the `FastBitShiftPIDController` is instantiated and configured for use. The `compute` method is then called within the main loop to continuously adjust the control output based on the process input and desired setpoint.
