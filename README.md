# kitronik-pico-robotics-i2c

C++ library for controlling the Kitronik Robotics Board using the
Raspberry Pi Pico SDK over I2C.

This library provides deterministic control of:

-   Servo motors (channels 1--8)
-   DC motors (channels 1--4)
-   Stepper motors (2 steppers via motor pairs)

using the PCA9685 PWM controller.

Designed for integration into embedded firmware projects targeting
RP2040 and RP2350.

------------------------------------------------------------------------

## Features

-   Native C++ implementation (no MicroPython runtime)
-   Deterministic PWM control via PCA9685
-   Compatible with Pico SDK
-   No dynamic allocation
-   No exceptions required
-   Real-time friendly
-   Per-servo calibration support (min/max pulse range)
-   Direct microsecond pulse control for precise positioning

------------------------------------------------------------------------

## Requirements

-   Raspberry Pi Pico SDK
-   hardware_i2c module

Supported targets:

-   RP2040 (Pico, Pico W)
-   RP2350 (Pico 2, Pico 2 W)

------------------------------------------------------------------------

## Installation

Add to your project:

    lib/
    └── kitronik_pico_robotics/

In your CMakeLists.txt:

``` cmake
add_subdirectory(lib/kitronik_pico_robotics)

target_link_libraries(app
    kitronik_pico_robotics
)
```

------------------------------------------------------------------------

## Example

``` cpp
#include "kitronik_pico_robotics.hpp"

KitronikPicoRobotics board(i2c0, 0x6C, 8, 9, 100000);

// Optional: calibrate servo range
board.setServoPulseRangeUs(1, 500, 2200);

// Move servo using degrees
board.servoWrite(1, 90);

// Direct pulse control (microseconds)
board.servoWriteUs(1, 1500);

// Motor control
board.motorOn(1, 'f', 50);

// Stepper control
board.step(1, 'f', 200, 20, false);
```

------------------------------------------------------------------------

## Servo Calibration

Different servo models may have different safe pulse ranges. The library
allows per-servo calibration.

Typical safe ranges:

-   Standard servos: 500--2500 µs
-   Some servos: 600--2400 µs
-   Others: may require custom values

Set per-servo range:

``` cpp
board.setServoPulseRangeUs(servo, min_us, max_us);
```

Set global range:

``` cpp
board.setAllServoPulseRangeUs(min_us, max_us);
```

Direct pulse control:

``` cpp
board.servoWriteUs(servo, pulse_us);
```

This allows precise positioning and prevents mechanical overtravel.

------------------------------------------------------------------------

## API

### Constructor

``` cpp
KitronikPicoRobotics(
    i2c_inst_t* i2c,
    uint8_t address,
    uint sda,
    uint scl,
    uint baudrate
);
```

### Servo

``` cpp
bool servoWrite(uint8_t servo, int degrees);
bool servoWriteRadians(uint8_t servo, float radians);
bool servoWriteUs(uint8_t servo, uint16_t pulse_us);

bool setServoPulseRangeUs(uint8_t servo, uint16_t min_us, uint16_t max_us);
void setAllServoPulseRangeUs(uint16_t min_us, uint16_t max_us);
```

### Motor

``` cpp
bool motorOn(uint8_t motor, char direction, int speed_percent);
bool motorOff(uint8_t motor);
```

### Stepper

``` cpp
bool step(uint8_t motor, char direction, int steps, int delay_ms, bool holdPosition);
bool stepAngle(uint8_t motor, char direction, float angle, int delay_ms, bool holdPosition, int stepsPerRev);
```

------------------------------------------------------------------------

## Hardware

Uses PCA9685 PWM controller via I2C.

Default address: `0x6C`

------------------------------------------------------------------------

## License

MIT
