#ifndef FASTBITSHIFTPIDCONTROLLER_H
#define FASTBITSHIFTPIDCONTROLLER_H

enum class BitShift : uint8_t {
    FAST_1 = 0,    // No scaling, multiply/divide by 1
    FAST_2 = 1,    // Multiply/divide by 2
    FAST_4 = 2,    // Multiply/divide by 4
    FAST_8 = 3,    // Multiply/divide by 8
    FAST_16 = 4,   // Multiply/divide by 16
    FAST_32 = 5,   // Multiply/divide by 32
    FAST_64 = 6,   // Multiply/divide by 64
    FAST_128 = 7,  // Multiply/divide by 128
    FAST_256 = 8,  // Multiply/divide by 256
    FAST_512 = 9,  // Multiply/divide by 512
    FAST_1024 = 10, // Multiply/divide by 1024
    FAST_2048 = 11, // Multiply/divide by 2048
    FAST_4096 = 12  // Multiply/divide by 4096
};

class FastBitShiftPIDController
{
  private:
    BitShift Kp_shift;
    BitShift Ki_shift;
    BitShift Kd_shift;
    BitShift scaleShift;
    int32_t integral;
    int32_t previousError;
    int32_t outputMin;
    int32_t outputMax;
    bool disableIntegral;
    bool disableDerivative;
    
    inline int32_t IntegralTerm(int32_t error)
    {
      if (disableIntegral) return 0;
      if (error == 0 || (error > 0 && integral < 0) || (error < 0 && integral > 0))  return 0;
     
      integral += error; // Accumulate integral term 
     

      // Apply bounds to prevent integral windup
      const int32_t integralMax = outputMax << static_cast<uint8_t>(scaleShift);
      if (integral > integralMax)
      {
        integral = integralMax;
      }
      else if (integral < -integralMax)
      {
        integral = -integralMax;
      }

      return integral;
    }

  public:
    FastBitShiftPIDController(BitShift Kp_shift, BitShift Ki_shift, BitShift Kd_shift, BitShift scaleShift = BitShift::FAST_1024, int32_t outputMin = -255, int32_t outputMax = 255)
    : Kp_shift(Kp_shift), Ki_shift(Ki_shift), Kd_shift(Kd_shift),
      scaleShift(scaleShift), integral(0), previousError(0),
      outputMin(outputMin), outputMax(outputMax),
      disableIntegral(false), disableDerivative(false)
    {
    }

     // Setter for Kp shift value
    void setKpShift(BitShift newKpShift) {
        Kp_shift = newKpShift;
    }

    // Setter for Ki shift value
    void setKiShift(BitShift newKiShift) {
        Ki_shift = newKiShift;
    }

    // Setter for Kd shift value
    void setKdShift(BitShift newKdShift) {
        Kd_shift = newKdShift;
    }

    // Optionally, if you need to set all PID terms at once
    void setPIDShifts(BitShift newKpShift, BitShift newKiShift, BitShift newKdShift) {
        setKpShift(newKpShift);
        setKiShift(newKiShift);
        setKdShift(newKdShift);
    }
    void setIntegralDisabled(bool disable)
    {
        disableIntegral = disable;
    }

    void setDerivativeDisabled(bool disable)
    {
        disableDerivative = disable;
    }

    int32_t compute(int32_t setpoint, int32_t input)
    {
        int32_t error = setpoint - input;

        // Integral term calculation
        if (!disableIntegral)
        {
            if (error == 0 || error > 0 && integral < 0 || error < 0 && integral > 0) 
	    {
               integral = 0;  // Reset integral term if setpoint is achieved or exceeded
	    }
            else
	    {
               integral += error;  // Accumulate integral term if not disabled
            }
            // Apply bounds to prevent integral windup
            int32_t integralMax = outputMax << static_cast<uint8_t>(scaleShift);
            if (integral > integralMax)
            {
                integral = integralMax;
            }
            else if (integral < -integralMax)
            {
                integral = -integralMax;
            }
        }
        else
        {
            integral = 0;  // Reset integral term if disabled
        }

        // PID terms calculation with consideration for disabled states
        int32_t pTerm = error << static_cast<uint8_t>(Kp_shift); // Proportional term
        int32_t iTerm = IntegralTerm(error); // Integral term
        int32_t dTerm = disableDerivative ? 0 : (error - previousError) << static_cast<uint8_t>(Kd_shift); // Derivative term

        // Combine the PID terms
        int32_t output = pTerm + iTerm + dTerm;
        output = output >> static_cast<uint8_t>(scaleShift);  // Apply scaling

        // Apply output limits
        if (output > outputMax)
        {
            output = outputMax;
        }
        else if (output < outputMin)
        {
            output = outputMin;
        }

        // Update the previous error for the next iteration
        previousError = error;

        return output;
    }
};

#endif // FASTBITSHIFTPIDCONTROLLER_H
