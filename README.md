# FastBitShiftPIDController Library

A fast PID controller that utilizes bit shifting for efficient computations. This is a header-only library for easy integration into projects.

## Installation

1. Download `FastBitShiftPIDController.h`.
2. Include it in your project directory.

## How to Use

Include the header in your source file and create an instance of the `FastBitShiftPIDController` class. Use the `compute` method with your setpoint and input values.

### Example:

```
#include "FastBitShiftPIDController.h"

int main() {
    FastBitShiftPIDController pid(10, 7, 4);
    int32_t setpoint = 100;
    int32_t input = 90;
    int32_t output = pid.compute(setpoint, input);
    // Use the output as needed...
}
```

## License

[Your preferred license, e.g., MIT, GPL, etc.]

## Contributing

If you have suggestions or improvements, please submit a pull request or open an issue. Contributions are welcome!
