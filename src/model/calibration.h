#ifndef EXCAVATOR_CALIBRATION_H
#define EXCAVATOR_CALIBRATION_H

#include "bmi160.h"
#include "config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    float angle_trim_rad[EXCAVATOR_IMU_COUNT];
} excavator_calibration_t;

void excavator_calibration_reset(excavator_calibration_t *calibration);
bool excavator_calibration_compute_zero_pose(excavator_calibration_t *calibration,
                                             const bmi160_sample_t *samples,
                                             const excavator_config_t *config);
float excavator_calibration_total_offset_rad(const excavator_calibration_t *calibration,
                                             const excavator_config_t *config,
                                             uint8_t sensor_index);

#endif
