#include "robot.h"
#include "cmsis_os.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

static void pca9685_write(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, reg, 1, data, len, HAL_MAX_DELAY);
}

void pca9685_init() {
    uint8_t mode1 = 0x10; // sleep
    pca9685_write(0x00, &mode1, 1);

    uint8_t prescale = 121; // ~50Hz
    pca9685_write(0xFE, &prescale, 1);

    mode1 = 0xA1; // auto-increment + restart
    pca9685_write(0x00, &mode1, 1);
}

static uint16_t angle_to_pwm(uint8_t angle) {
    // Wider range: ~0.5ms to 2.5ms (102 to 512 PWM values)
    // This matches the 500-2500µs range commonly used
    return (uint16_t)(102 + ((float)angle / 180.0f) * (512 - 102));
}

void set_servo(uint8_t channel, uint8_t angle) {
    uint16_t pwm = angle_to_pwm(angle);
    uint8_t reg = 0x06 + 4 * channel;
    uint8_t data[4] = {0x00, 0x00, pwm & 0xFF, pwm >> 8};

    pca9685_write(reg, data, 4);
}
