#include "excavator_state.h"

static gps_fix_t excavator_state_reference_fix(const excavator_state_t *state)
{
    if ((state == 0) || !state->gps_reference.valid) {
        return (gps_fix_t){
            .valid = false,
            .altitude_m = 0.0f,
        };
    }

    return (gps_fix_t){
        .valid = true,
        .altitude_m = state->gps_reference.altitude_m,
    };
}

void excavator_state_init(excavator_state_t *state, const excavator_config_t *config)
{
    uint8_t index;

    if (state == 0) {
        return;
    }

    *state = (excavator_state_t){
        .config = (config != 0) ? *config : excavator_config_default(),
        .platform_status = {
            .backend = PLATFORM_BACKEND_SIMULATED,
            .ready_flags = 0u,
            .error_flags = PLATFORM_ERROR_NOT_IMPLEMENTED,
        },
        .gps_reference = {
            .valid = false,
            .altitude_m = 0.0f,
        },
        .calibration = {
            .zero_pose = {
                .valid = false,
                .angle_trim_rad = {0.0f, 0.0f, 0.0f, 0.0f},
            },
        },
        .estimation = {
            .valid = false,
            .orientation = orientation_estimate_level(),
        },
        .result = {
            .valid = false,
            .bucket_height = {
                .inputs_valid = false,
                .delta_height_m = 0.0f,
                .absolute_height_m = 0.0f,
            },
        },
    };

    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        state->raw.valid[index] = false;
        state->raw.imu[index] = (bmi160_sample_t){0};
    }
}

bool excavator_state_set_imu_sample(excavator_state_t *state,
                                    excavator_sensor_id_t sensor,
                                    const bmi160_sample_t *sample)
{
    if ((state == 0) || (sample == 0) || (sensor >= EXCAVATOR_SENSOR_COUNT)) {
        return false;
    }

    state->raw.imu[sensor] = *sample;
    state->raw.valid[sensor] = true;
    return true;
}

void excavator_state_set_gps_fix(excavator_state_t *state, const gps_fix_t *fix)
{
    if (state == 0) {
        return;
    }

    if ((fix == 0) || !fix->valid) {
        state->gps_reference.valid = false;
        state->gps_reference.altitude_m = 0.0f;
        return;
    }

    state->gps_reference.valid = true;
    state->gps_reference.altitude_m = fix->altitude_m;
}

void excavator_state_set_orientation(excavator_state_t *state, const orientation_estimate_t *orientation)
{
    if (state == 0) {
        return;
    }

    if (orientation == 0) {
        state->estimation.valid = false;
        state->estimation.orientation = orientation_estimate_level();
        return;
    }

    state->estimation.valid = true;
    state->estimation.orientation = *orientation;
}

void excavator_state_set_platform_status(excavator_state_t *state, platform_status_t status)
{
    if (state == 0) {
        return;
    }

    state->platform_status = status;
}

bool excavator_state_calibrate_zero_pose(excavator_state_t *state)
{
    if (state == 0) {
        return false;
    }

    return excavator_calibration_compute_zero_pose(&state->calibration.zero_pose, state->raw.imu, &state->config);
}

bool excavator_state_estimate_orientation_quasistatic(excavator_state_t *state)
{
    orientation_estimate_t orientation;

    if (state == 0) {
        return false;
    }

    if (!state->raw.valid[EXCAVATOR_SENSOR_S1] ||
        !state->raw.valid[EXCAVATOR_SENSOR_S2] ||
        !state->raw.valid[EXCAVATOR_SENSOR_S3] ||
        !state->raw.valid[EXCAVATOR_SENSOR_S4]) {
        state->estimation.valid = false;
        return false;
    }

    if (!orientation_quasistatic_sample_valid(&state->raw.imu[EXCAVATOR_SENSOR_S1], &state->config.quasistatic_filter) ||
        !orientation_quasistatic_sample_valid(&state->raw.imu[EXCAVATOR_SENSOR_S2], &state->config.quasistatic_filter) ||
        !orientation_quasistatic_sample_valid(&state->raw.imu[EXCAVATOR_SENSOR_S3], &state->config.quasistatic_filter) ||
        !orientation_quasistatic_sample_valid(&state->raw.imu[EXCAVATOR_SENSOR_S4], &state->config.quasistatic_filter)) {
        state->estimation.valid = false;
        return false;
    }

    orientation = (orientation_estimate_t){
        .s1_angle_rad = orientation_quasistatic_angle_from_accel(
            &state->raw.imu[EXCAVATOR_SENSOR_S1],
            excavator_calibration_total_offset_rad(&state->calibration.zero_pose, &state->config, EXCAVATOR_SENSOR_S1)),
        .s2_angle_rad = orientation_quasistatic_angle_from_accel(
            &state->raw.imu[EXCAVATOR_SENSOR_S2],
            excavator_calibration_total_offset_rad(&state->calibration.zero_pose, &state->config, EXCAVATOR_SENSOR_S2)),
        .s3_angle_rad = orientation_quasistatic_angle_from_accel(
            &state->raw.imu[EXCAVATOR_SENSOR_S3],
            excavator_calibration_total_offset_rad(&state->calibration.zero_pose, &state->config, EXCAVATOR_SENSOR_S3)),
        .s4_angle_rad = orientation_quasistatic_angle_from_accel(
            &state->raw.imu[EXCAVATOR_SENSOR_S4],
            excavator_calibration_total_offset_rad(&state->calibration.zero_pose, &state->config, EXCAVATOR_SENSOR_S4)),
    };

    state->estimation.valid = true;
    state->estimation.orientation = orientation;
    return true;
}

bool excavator_state_update_result(excavator_state_t *state)
{
    const gps_fix_t gps_fix = excavator_state_reference_fix(state);
    const orientation_estimate_t *orientation =
        (state != 0 && state->estimation.valid) ? &state->estimation.orientation : 0;

    if (state == 0) {
        return false;
    }

    state->result.bucket_height =
        kinematics_calculate_bucket_height(&state->config.geometry, orientation, &gps_fix);
    state->result.valid = state->result.bucket_height.inputs_valid;
    return state->result.valid;
}

bool excavator_state_inputs_ready(const excavator_state_t *state)
{
    uint8_t index;

    if (state == 0) {
        return false;
    }

    if (!platform_status_ok(state->platform_status) ||
        !excavator_geometry_is_configured(&state->config.geometry) ||
        !state->gps_reference.valid ||
        !state->estimation.valid) {
        return false;
    }

    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        if (!state->raw.valid[index]) {
            return false;
        }
    }

    return true;
}

bool excavator_state_output_ready(const excavator_state_t *state)
{
    if (state == 0) {
        return false;
    }

    return platform_status_ok(state->platform_status) &&
           state->estimation.valid &&
           state->result.valid;
}
