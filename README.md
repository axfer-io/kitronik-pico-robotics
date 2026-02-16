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

board.servoWrite(1, 90);
board.motorOn(1, 'f', 50);
board.step(1, 'f', 200, 20, false);
```

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
void servoWrite(uint8_t servo, uint16_t degrees);
void servoWriteRadians(uint8_t servo, float radians);
```

### Motor

``` cpp
void motorOn(uint8_t motor, char direction, uint8_t speed);
void motorOff(uint8_t motor);
```

### Stepper

``` cpp
void step(uint8_t motor, char direction, uint16_t steps, uint delay_ms, bool holdPosition);
void stepAngle(uint8_t motor, char direction, float angle, uint delay_ms, bool holdPosition, uint stepsPerRev);
```

------------------------------------------------------------------------

## Hardware

Uses PCA9685 PWM controller via I2C.

Default address: `0x6C`

------------------------------------------------------------------------

## License

MIT
