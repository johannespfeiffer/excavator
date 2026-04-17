#ifndef EXCAVATOR_STATE_H
#define EXCAVATOR_STATE_H

#include "bmi160.h"
#include "config.h"
#include "gps_parser.h"
#include "kinematics.h"
#include "orientation.h"
#include "platform.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EXCAVATOR_SENSOR_S1 = 0,
    EXCAVATOR_SENSOR_S2 = 1,
    EXCAVATOR_SENSOR_S3 = 2,
    EXCAVATOR_SENSOR_S4 = 3,
    EXCAVATOR_SENSOR_COUNT = EXCAVATOR_IMU_COUNT,
} excavator_sensor_id_t;

typedef struct {
    bool valid[EXCAVATOR_SENSOR_COUNT];
    bmi160_sample_t imu[EXCAVATOR_SENSOR_COUNT];
} excavator_raw_data_t;

typedef struct {
    bool valid;
    float altitude_m;
} excavator_reference_height_t;

typedef struct {
    bool valid;
    orientation_estimate_t orientation;
} excavator_estimation_t;

typedef struct {
    bool valid;
    bucket_height_result_t bucket_height;
} excavator_result_data_t;

typedef struct {
    excavator_config_t config;
    platform_status_t platform_status;
    excavator_raw_data_t raw;
    excavator_reference_height_t gps_reference;
    excavator_estimation_t estimation;
    excavator_result_data_t result;
} excavator_state_t;

void excavator_state_init(excavator_state_t *state, const excavator_config_t *config);
bool excavator_state_set_imu_sample(excavator_state_t *state,
                                    excavator_sensor_id_t sensor,
                                    const bmi160_sample_t *sample);
void excavator_state_set_gps_fix(excavator_state_t *state, const gps_fix_t *fix);
void excavator_state_set_orientation(excavator_state_t *state, const orientation_estimate_t *orientation);
void excavator_state_set_platform_status(excavator_state_t *state, platform_status_t status);
bool excavator_state_estimate_orientation_quasistatic(excavator_state_t *state);
bool excavator_state_update_result(excavator_state_t *state);
bool excavator_state_inputs_ready(const excavator_state_t *state);
bool excavator_state_output_ready(const excavator_state_t *state);

#endif
