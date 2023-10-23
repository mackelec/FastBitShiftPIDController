# FastBitShiftPIDController
PID Controller for Arduino that is optimized for Speed by use bit Shifting instead of multiply and divide.


```
#include <cstdint>

class FastBitShiftPIDController {
private:
    // Proportional gain shift value (2^Kp_shift).
    // Accelerates the proportional term calculation using bit shifting.
    // Larger values increase the scaling factor (e.g., 2^10 = 1024).
    uint8_t Kp_shift;

    // Integral gain shift value (2^Ki_shift).
    // Speeds up the integral term calculation using bit shifting.
    // Larger values increase the scaling factor (e.g., 2^7 = 128).
    uint8_t Ki_shift;

    // Derivative gain shift value (2^Kd_shift).
    // Enhances the derivative term computation using bit shifting.
    // Larger values amplify the scaling factor (e.g., 2^4 = 16).
    uint8_t Kd_shift;

    // Overall scaling shift value (2^scaleShift).
    // Applies scaling to error, integral, and derivative terms for precision control.
    // Larger values increase the scaling factor (e.g., 2^10 = 1024).
    int32_t scaleShift;

    int32_t integral;
    int32_t previousError;
    int32_t outputMin;
    int32_t outputMax;

public:
    // Constructor to initialize PID controller parameters.
    // Kp_shift, Ki_shift, and Kd_shift determine the response of the controller.
    // scaleShift scales the error and integral terms to avoid overflow.
    FastBitShiftPIDController(uint8_t Kp_shift, uint8_t Ki_shift, uint8_t Kd_shift, int32_t scaleShift = 10, int32_t outputMin = -255, int32_t outputMax = 255)
        : Kp_shift(Kp_shift), Ki_shift(Ki_shift), Kd_shift(Kd_shift),
          scaleShift(scaleShift), integral(0), previousError(0),
          outputMin(outputMin), outputMax(outputMax) {}

    int32_t compute(int32_t setpoint, int32_t input) {
        int32_t error = (setpoint - input) << scaleShift;  // Scale the error

        if (error == 0 || error > 0 && integral > 0 || error < 0 && integral < 0) {
            integral = 0;  // Reset integral term if setpoint is achieved or exceeded
        } else {
            integral += error;
        }

        // Apply bounds to prevent integral windup
        int32_t integralMax = outputMax << scaleShift;
        if (integral > integralMax) {
            integral = integralMax;
        } else if (integral < -integralMax) {
            integral = -integralMax;
        }

        int32_t pTerm = error << Kp_shift;  // Scale the proportional term
        int32_t iTerm = integral << Ki_shift;  // Scale the integral term
        int32_t dTerm = (error - previousError) << Kd_shift;  // Scale the derivative term

        int32_t output = pTerm + iTerm + dTerm;
        output = output >> scaleShift;  // Undo the overall scaling

        // Apply output limits
        if (output > outputMax) {
            output = outputMax;
        } else if (output < outputMin) {
            output = outputMin;
        }

        // Update the previous error for the next iteration
        previousError = error;

        return output;
    }
};

```

###  Example

```

int main() {
    // Define the PID controller parameters.
    uint8_t Kp_shift = 10;  // Proportional gain shift (2^10 = 1024)
    uint8_t Ki_shift = 7;   // Integral gain shift (2^7 = 128)
    uint8_t Kd_shift = 4;   // Derivative gain shift (2^4 = 16)
    int32_t scaleShift = 10; // Scaling factor (2^10 = 1024)
    int32_t outputMin = -255; // Minimum output limit
    int32_t outputMax = 255;  // Maximum output limit

    // Create a PID controller instance.
    FastBitShiftPIDController pidController(Kp_shift, Ki_shift, Kd_shift, scaleShift, outputMin, outputMax);

    // Simulate a control loop.
    int32_t setpoint = 100;  // Desired setpoint
    int32_t input = 0;       // Current system input (initially at 0)
    int32_t output = 0;      // Controller output

    for (int i = 0; i < 100; ++i) {
        // Compute the controller output.
        output = pidController.compute(setpoint, input);

        // Update the system input (in a real application, this would come from your system).
        input += (output / 10);  // Simulated change in the system input.

        // Print the results.
        Serial.print("Iteration ");
        Serial.print(i);
        Serial.print(": Setpoint = ");
        Serial.print(setpoint);
        Serial.print(", Input = ");
        Serial.print(input);
        Serial.print(", Output = ");
        Serial.println(output);
    }

    return 0;
}

```
