#ifndef FASTBITSHIFTPIDCONTROLLER_H
#define FASTBITSHIFTPIDCONTROLLER_H

//#include <cstdint>

class FastBitShiftPIDController {
private:
    uint8_t Kp_shift;
    uint8_t Ki_shift;
    uint8_t Kd_shift;
    int32_t scaleShift;
    int32_t integral;
    int32_t previousError;
    int32_t outputMin;
    int32_t outputMax;

public:
    FastBitShiftPIDController(uint8_t Kp_shift, uint8_t Ki_shift, uint8_t Kd_shift, int32_t scaleShift = 10, int32_t outputMin = -255, int32_t outputMax = 255)
        : Kp_shift(Kp_shift), Ki_shift(Ki_shift), Kd_shift(Kd_shift),
          scaleShift(scaleShift), integral(0), previousError(0),
          outputMin(outputMin), outputMax(outputMax) {}

    int32_t compute(int32_t setpoint, int32_t input) {
        int32_t error = (setpoint - input) << scaleShift;  // Scale the error

        if (error == 0 || error > 0 && integral < 0 || error < 0 && integral > 0) {
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

#endif // FASTBITSHIFTPIDCONTROLLER_H
