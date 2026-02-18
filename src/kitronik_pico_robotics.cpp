#include "kitronik_pico_robotics.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cmath>

KitronikPicoRobotics::KitronikPicoRobotics(i2c_inst_t* i2c, uint8_t addr, uint sda, uint scl, uint32_t baudrate)
: i2c_(i2c), addr_(addr) {
    i2c_init(i2c_, baudrate);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    for (int i = 0; i < 8; ++i) {
        servo_min_us_[i] = SERVO_MIN_US_DEFAULT;
        servo_max_us_[i] = SERVO_MAX_US_DEFAULT;
    }
    initPCA();
}

bool KitronikPicoRobotics::softwareReset() {
    // MicroPython: i2c.writeto(0, "\x06")  -> General Call, SWRST=0x06
    uint8_t buf[1] = {0x06};
    int rc = i2c_write_blocking(i2c_, 0x00, buf, 1, false);
    return (rc == 1);
}

bool KitronikPicoRobotics::write8(uint8_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int rc = i2c_write_blocking(i2c_, dev, buf, 2, false);
    return (rc == 2);
}

bool KitronikPicoRobotics::writeBlock(uint8_t dev, uint8_t reg, const uint8_t* data, size_t len) {
    // reg + payload
    // Ojo: para len pequeño (1..4) va fino; si quieres, lo haces con buffer fijo.
    uint8_t tmp[1 + 16];
    if (len > 16) return false;
    tmp[0] = reg;
    for (size_t i = 0; i < len; ++i) tmp[1 + i] = data[i];
    int rc = i2c_write_blocking(i2c_, dev, tmp, 1 + (int)len, false);
    return (rc == (int)(1 + len));
}

bool KitronikPicoRobotics::initPCA() {
    // Soft reset para arrancar “limpio”
    (void)softwareReset();

    // Prescale para ~50Hz (20ms)
    if (!write8(addr_, PRESCALE, prescale_val_)) return false;

    // Apaga ALL outputs (OFF=0)
    if (!write8(addr_, ALL_LED_ON_L,  0x00)) return false;
    if (!write8(addr_, ALL_LED_ON_H,  0x00)) return false;
    if (!write8(addr_, ALL_LED_OFF_L, 0x00)) return false;
    if (!write8(addr_, ALL_LED_OFF_H, 0x00)) return false;

    // MODE1 = 0x01 (sale de sleep; tu código hace writeto_mem(0x00,"\x01"))
    if (!write8(addr_, MODE1, 0x01)) return false;

    sleep_us(500);
    return true;
}

bool KitronikPicoRobotics::adjustServos(int change) {
    if (change < -25) change = -25;
    if (change >  25) change =  25;
    prescale_val_ = (uint8_t)(121 + change);
    return initPCA();
}

bool KitronikPicoRobotics::servoWrite(uint8_t servo, int degrees) {
    if (servo < 1 || servo > 8) return false;
    const uint16_t pulse_us = degrees_to_us_(servo, degrees);
    return servoWriteUs(servo, pulse_us);
}

bool KitronikPicoRobotics::servoWriteRadians(uint8_t servo, float radians) {
    if (servo < 1 || servo > 8) return false;
    const uint16_t pulse_us = radians_to_us_(servo, radians);
    return servoWriteUs(servo, pulse_us);
}

bool KitronikPicoRobotics::motorOn(uint8_t motor, char direction, int speed_percent) {
    if (speed_percent < 0)   speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;
    if (motor < 1 || motor > 4) return false;

    uint8_t motorReg = MOT_REG_BASE + (uint8_t)(2 * (motor - 1) * REG_OFFSET);

    // PWMVal = int(speed * 40.95)
    int pwm = (int)(speed_percent * 40.95f);
    uint8_t low  = (uint8_t)(pwm & 0xFF);
    uint8_t high = (uint8_t)((pwm >> 8) & 0x0F);

    auto zeroPair = [&]() -> bool {
        if (!write8(addr_, motorReg,   0)) return false;
        if (!write8(addr_, motorReg+1, 0)) return false;
        if (!write8(addr_, motorReg+4, 0)) return false;
        if (!write8(addr_, motorReg+5, 0)) return false;
        return true;
    };

    if (direction == 'f') {
        if (!write8(addr_, motorReg,   low))  return false;
        if (!write8(addr_, motorReg+1, high)) return false;
        if (!write8(addr_, motorReg+4, 0))    return false;
        if (!write8(addr_, motorReg+5, 0))    return false;
        return true;
    } else if (direction == 'r') {
        if (!write8(addr_, motorReg+4, low))  return false;
        if (!write8(addr_, motorReg+5, high)) return false;
        if (!write8(addr_, motorReg,   0))    return false;
        if (!write8(addr_, motorReg+1, 0))    return false;
        return true;
    }

    (void)zeroPair();
    return false;
}

bool KitronikPicoRobotics::motorOff(uint8_t motor) {
    return motorOn(motor, 'f', 0);
}

bool KitronikPicoRobotics::step(uint8_t motor, char direction, int steps, int speed_ms, bool holdPosition) {
    if (motor < 1 || motor > 2) return false;
    if (steps <= 0) return true;

    char directions[2];
    uint8_t coils[2];

    if (direction == 'f') {
        directions[0] = 'f'; directions[1] = 'r';
        coils[0] = (uint8_t)((motor * 2) - 1);
        coils[1] = (uint8_t)(motor * 2);
    } else if (direction == 'r') {
        directions[0] = 'r'; directions[1] = 'f';
        coils[0] = (uint8_t)(motor * 2);
        coils[1] = (uint8_t)((motor * 2) - 1);
    } else {
        return false;
    }

    while (steps > 0) {
        for (int di = 0; di < 2 && steps > 0; ++di) {
            for (int ci = 0; ci < 2 && steps > 0; ++ci) {
                if (!motorOn(coils[ci], directions[di], 100)) return false;
                sleep_ms(speed_ms);
                steps--;
            }
        }
    }

    if (!holdPosition) {
        (void)motorOff(coils[0]);
        (void)motorOff(coils[1]);
    }
    return true;
}

bool KitronikPicoRobotics::stepAngle(uint8_t motor, char direction, float angle_deg, int speed_ms, bool holdPosition, int stepsPerRev) {
    if (stepsPerRev <= 0) return false;
    int steps = (int)(angle_deg / (360.0f / (float)stepsPerRev));
    return step(motor, direction, steps, speed_ms, holdPosition);
}

float KitronikPicoRobotics::pca_pwm_freq_hz_() const {
    // PCA9685 datasheet: freq ≈ osc / (4096 * (prescale+1))
    return PCA9685_OSC_HZ / (4096.0f * (float)(prescale_val_ + 1));
}

uint16_t KitronikPicoRobotics::us_to_counts_(uint16_t pulse_us) const {
    // counts = pulse_us / period_us * 4096
    const float freq = pca_pwm_freq_hz_();
    const float period_us = 1000000.0f / freq;
    float counts_f = ((float)pulse_us * 4096.0f) / period_us;

    int counts = (int)lroundf(counts_f);
    if (counts < 0) counts = 0;
    if (counts > 4095) counts = 4095;
    return (uint16_t)counts;
}

uint16_t KitronikPicoRobotics::degrees_to_us_(uint8_t servo, int degrees) const {
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;

    const int idx = (int)servo - 1;
    if (idx < 0 || idx >= 8) return SERVO_MIN_US_DEFAULT;
    const float min_us = (float)servo_min_us_[idx];
    const float max_us = (float)servo_max_us_[idx];

    // linear map 0..180 -> min..max
    return (uint16_t)lroundf(min_us + ((float)degrees / 180.0f) * (max_us - min_us));
}

uint16_t KitronikPicoRobotics::radians_to_us_(uint8_t servo, float radians) const {
    if (radians < 0.0f) radians = 0.0f;
    if (radians > PI_ESTIMATE) radians = PI_ESTIMATE;

    const int idx = (int)servo - 1;
    if (idx < 0 || idx >= 8) return SERVO_MIN_US_DEFAULT;
    const float min_us = (float)servo_min_us_[idx];
    const float max_us = (float)servo_max_us_[idx];

    return (uint16_t)lroundf(min_us + (radians / PI_ESTIMATE) * (max_us - min_us));
}

bool KitronikPicoRobotics::setServoPulseRangeUs(uint8_t servo, uint16_t min_us, uint16_t max_us) {
    if (servo < 1 || servo > 8) return false;

    // Sane clamps (puedes ajustar si quieres)
    if (min_us < 400)  min_us = 400;
    if (max_us > 2600) max_us = 2600;
    if (min_us >= max_us) return false;

    const int idx = (int)servo - 1;
    servo_min_us_[idx] = min_us;
    servo_max_us_[idx] = max_us;
    return true;
}

void KitronikPicoRobotics::setAllServoPulseRangeUs(uint16_t min_us, uint16_t max_us) {
    // reuse validation logic loosely
    if (min_us < 400)  min_us = 400;
    if (max_us > 2600) max_us = 2600;
    if (min_us >= max_us) { min_us = SERVO_MIN_US_DEFAULT; max_us = SERVO_MAX_US_DEFAULT; }

    for (int i = 0; i < 8; ++i) {
        servo_min_us_[i] = min_us;
        servo_max_us_[i] = max_us;
    }
}

bool KitronikPicoRobotics::servoWriteUs(uint8_t servo, uint16_t pulse_us) {
    if (servo < 1 || servo > 8) return false;

    const int idx = (int)servo - 1;
    // clamp to that servo's calibrated range
    if (pulse_us < servo_min_us_[idx]) pulse_us = servo_min_us_[idx];
    if (pulse_us > servo_max_us_[idx]) pulse_us = servo_max_us_[idx];

    const uint8_t calcServo = SRV_REG_BASE + (uint8_t)((servo - 1) * REG_OFFSET);
    const uint16_t pwm = us_to_counts_(pulse_us);

    const uint8_t low  = (uint8_t)(pwm & 0xFF);
    const uint8_t high = (uint8_t)((pwm >> 8) & 0x03); // PCA uses 12-bit, tu protocolo hoy manda 1 bit, pero mejor 0x0F

    if (!write8(addr_, calcServo,     low))  return false;
    if (!write8(addr_, calcServo + 1, high)) return false;
    return true;
}


