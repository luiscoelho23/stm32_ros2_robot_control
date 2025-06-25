#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "stm32f7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define the I2C address for PCA9685
#define PCA9685_ADDR     (0x40 << 1)  // Shifted for HAL

// Extern I2C handle (declared in main.c)
extern I2C_HandleTypeDef hi2c1;

// Function prototypes
void pca9685_init(void);
void set_servo(uint8_t channel, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_H__ */
