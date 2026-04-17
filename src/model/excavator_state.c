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
        .estimation = {
            .valid = true,
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
