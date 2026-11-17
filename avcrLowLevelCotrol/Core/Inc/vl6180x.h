#ifndef VL6180X_H
#define VL6180X_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h" // Replace with your specific family header (e.g., stm32l4xx_hal.h)

/* Comment these out if not connected */
#define VL6180X_SECOND
#define VL6180X_THIRD
#define VL6180X_FORTH

#define VL6180X_OUT_OF_RANGE (255)



#define CLIFF_DIS 	160 // 160mm = 16cm


typedef enum
{
    VL6180X_IDX_FIRST,
#ifdef VL6180X_SECOND
    VL6180X_IDX_SECOND,
#endif
#ifdef VL6180X_THIRD
    VL6180X_IDX_THIRD,
#endif
#ifdef VL6180X_FORTH
	VL6180X_IDX_FORTH,
#endif
} vl6180x_idx_t;


typedef struct {
	uint8_t distanceSensor1;
	uint8_t distanceSensor2;
	uint8_t distanceSensor3;
	uint8_t distanceSensor4;
}allDistanceTofData_t;

typedef enum
{
	cliff_not_detected = 0,
	cliff_detected = 1
}cliff_detect;
typedef enum
{
	init_failed= 0,
	init_success = 1
}init_status;

typedef struct {
	uint8_t cliff1;
	uint8_t cliff2;
	uint8_t cliff3;
	uint8_t cliff4;
}cliffs_status_t;


/**
 * Initializes the sensors in the vl6180x_idx_t enum.
 * @param hi2c Pointer to the initialized STM32 I2C HAL handle.
 */
bool vl6180x_init(I2C_HandleTypeDef *hi2c);

/**
 * Does a single range measurement
 * @param idx selects specific sensor
 * @param range contains the measured range or VL6180X_OUT_OF_RANGE if out of range.
 * @return True if success, False if error
 */
bool vl6180x_read_range_single(vl6180x_idx_t idx, uint8_t *range);

void readAllvl6180xData(allDistanceTofData_t *sensorsData);
bool updateCliffStatus(allDistanceTofData_t *sensorsData , cliffs_status_t * cliffDetected);



#endif /* VL6180X_H */
