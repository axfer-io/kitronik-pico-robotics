#pragma once
#include <cstdint>
#include "hardware/i2c.h"

class KitronikPicoRobotics {
public:
    // Defaults: addr=0x6C, I2C0, SDA=8, SCL=9 (como tu MicroPython)
    KitronikPicoRobotics(i2c_inst_t* i2c = i2c0, uint8_t addr = 0x6C, uint sda = 8, uint scl = 9, uint32_t baudrate = 100000);

    // Core
    bool initPCA();
    bool adjustServos(int change);

    // Servos (1..8)
    bool servoWrite(uint8_t servo, int degrees);
    bool servoWriteRadians(uint8_t servo, float radians);

    // DC motors (1..4). direction: 'f' forward, 'r' reverse
    bool motorOn(uint8_t motor, char direction, int speed_percent);
    bool motorOff(uint8_t motor);

    // Steppers: motor=1 or 2 (usa pares de motores DC como bobinas)
    bool step(uint8_t motor, char direction, int steps, int speed_ms = 20, bool holdPosition = false);
    bool stepAngle(uint8_t motor, char direction, float angle_deg, int speed_ms = 20, bool holdPosition = false, int stepsPerRev = 200);
    
    // Servo calibration (per channel 1..8)
    bool setServoPulseRangeUs(uint8_t servo, uint16_t min_us, uint16_t max_us);
    void setAllServoPulseRangeUs(uint16_t min_us, uint16_t max_us);

    // Optional: direct pulse write (good for calibration)
    bool servoWriteUs(uint8_t servo, uint16_t pulse_us);


private:
    // Helpers I2C
    bool write8(uint8_t dev, uint8_t reg, uint8_t val);
    bool writeBlock(uint8_t dev, uint8_t reg, const uint8_t* data, size_t len);
    bool softwareReset(); // General Call reset

private:
    i2c_inst_t* i2c_;
    uint8_t addr_;

    // Constantes de tu lib
    static constexpr uint8_t SRV_REG_BASE = 0x08;
    static constexpr uint8_t MOT_REG_BASE = 0x28;
    static constexpr uint8_t REG_OFFSET  = 4;

    // PCA9685 regs útiles
    static constexpr uint8_t MODE1    = 0x00;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t ALL_LED_ON_L  = 0xFA;
    static constexpr uint8_t ALL_LED_ON_H  = 0xFB;
    static constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;

    // Servo mapping de tu código
    uint8_t prescale_val_ = 0x79;        // 121 decimal = 0x79
    static constexpr float PI_ESTIMATE = 3.1416f;
    
    // Servo per-channel ranges (defaults 500..2500)
    uint16_t servo_min_us_[8];
    uint16_t servo_max_us_[8];

    static constexpr uint16_t SERVO_MIN_US_DEFAULT = 500;
    static constexpr uint16_t SERVO_MAX_US_DEFAULT = 2500;

    // PCA9685 oscillator (typical). Used to compute PWM frequency from prescale.
    static constexpr float PCA9685_OSC_HZ = 25000000.0f;

    // Helpers
    float pca_pwm_freq_hz_() const;
    uint16_t us_to_counts_(uint16_t pulse_us) const;
    uint16_t degrees_to_us_(uint8_t servo, int degrees) const;
    uint16_t radians_to_us_(uint8_t servo, float radians) const;

};
