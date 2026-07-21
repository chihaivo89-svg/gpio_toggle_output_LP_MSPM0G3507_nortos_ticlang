#include "heading_control.h"

#include <stdint.h>

#include "heading_control_config.h"
#include "speed_control.h"
#include "vehicle_yaw.h"

static bool s_active;
static float s_referenceDeg;
static float s_headingDeg;
static float s_errorDeg;
static float s_yawRateDps;
static float s_filteredErrorDeg;
static float s_filteredYawRateDps;
static float s_targetOffset;
static uint8_t s_correctionZone;

static float HeadingControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float HeadingControl_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float HeadingControl_MoveToward(
    float current,
    float target,
    float step)
{
    if (current < target) {
        float next = current + step;
        return (next > target) ? target : next;
    }
    if (current > target) {
        float next = current - step;
        return (next < target) ? target : next;
    }
    return current;
}

static void HeadingControl_UpdateObservation(void)
{
    s_headingDeg = VehicleYaw_GetHeadingDeg();
    s_yawRateDps = VehicleYaw_GetRateDps();
    s_errorDeg = s_headingDeg - s_referenceDeg;
}

/*
 * Hysteresis keeps small floor and IMU disturbances quiet.  The active zone
 * limits are differential speed targets: +/-1, +/-2 or +/-3 pulses/20 ms.
 */
static void HeadingControl_UpdateZone(float absoluteError)
{
    float exitDeadband = HEADING_CONTROL_DEADBAND_DEG * 0.5f;

    switch (s_correctionZone) {
        case 0U:
            if (absoluteError >= HEADING_CONTROL_STRONG_ERROR_DEG) {
                s_correctionZone = 3U;
            } else if (absoluteError >= HEADING_CONTROL_MEDIUM_ERROR_DEG) {
                s_correctionZone = 2U;
            } else if (absoluteError >= HEADING_CONTROL_DEADBAND_DEG) {
                s_correctionZone = 1U;
            }
            break;

        case 1U:
            if (absoluteError >= HEADING_CONTROL_STRONG_ERROR_DEG) {
                s_correctionZone = 3U;
            } else if (absoluteError >= HEADING_CONTROL_MEDIUM_ERROR_DEG) {
                s_correctionZone = 2U;
            } else if (absoluteError <= exitDeadband) {
                s_correctionZone = 0U;
            }
            break;

        case 2U:
            if (absoluteError >= HEADING_CONTROL_STRONG_ERROR_DEG) {
                s_correctionZone = 3U;
            } else if (absoluteError <=
                       HEADING_CONTROL_MEDIUM_RELEASE_DEG) {
                s_correctionZone = (absoluteError <= exitDeadband) ? 0U : 1U;
            }
            break;

        default:
            if (absoluteError <= HEADING_CONTROL_STRONG_RELEASE_DEG) {
                if (absoluteError <= exitDeadband) {
                    s_correctionZone = 0U;
                } else if (absoluteError >=
                           HEADING_CONTROL_MEDIUM_ERROR_DEG) {
                    s_correctionZone = 2U;
                } else {
                    s_correctionZone = 1U;
                }
            }
            break;
    }
}

static void HeadingControl_ResetOutput(void)
{
    s_targetOffset = 0.0f;
    s_correctionZone = 0U;
    SpeedControl_SetDifferentialTargetOffsetFloat(0.0f);
}

void HeadingControl_Init(void)
{
    s_active = false;
    s_referenceDeg = 0.0f;
    s_headingDeg = 0.0f;
    s_errorDeg = 0.0f;
    s_yawRateDps = 0.0f;
    s_filteredErrorDeg = 0.0f;
    s_filteredYawRateDps = 0.0f;
    HeadingControl_ResetOutput();
}

bool HeadingControl_Start(float referenceDeg)
{
    if (!VehicleYaw_IsCalibrated() || !SpeedControl_IsRunning()) {
        return false;
    }

    s_referenceDeg = referenceDeg;
    HeadingControl_UpdateObservation();
    s_filteredErrorDeg = s_errorDeg;
    s_filteredYawRateDps = s_yawRateDps;
    s_active = true;
    HeadingControl_ResetOutput();
    return true;
}

void HeadingControl_Stop(void)
{
    s_active = false;
    s_filteredErrorDeg = 0.0f;
    s_filteredYawRateDps = 0.0f;
    HeadingControl_ResetOutput();
}

void HeadingControl_Update20ms(void)
{
    float absoluteError;
    float command = 0.0f;
    float zoneLimit;

    if (!s_active) {
        return;
    }

    if (!VehicleYaw_IsCalibrated() || !SpeedControl_IsRunning()) {
        HeadingControl_Stop();
        return;
    }

    HeadingControl_UpdateObservation();
    s_filteredErrorDeg +=
        (s_errorDeg - s_filteredErrorDeg) *
        HEADING_CONTROL_ERROR_FILTER_ALPHA;
    s_filteredYawRateDps +=
        (s_yawRateDps - s_filteredYawRateDps) *
        HEADING_CONTROL_YAW_RATE_FILTER_ALPHA;

    absoluteError = HeadingControl_Abs(s_filteredErrorDeg);
    HeadingControl_UpdateZone(absoluteError);

    if (s_correctionZone != 0U) {
        /* Direction was verified on the real vehicle: left is positive yaw. */
        command = -(HEADING_CONTROL_KP * s_filteredErrorDeg +
                    HEADING_CONTROL_KD * s_filteredYawRateDps);
    }

    zoneLimit = (float)s_correctionZone;
    if (zoneLimit > (float)HEADING_CONTROL_MAX_TARGET_OFFSET) {
        zoneLimit = (float)HEADING_CONTROL_MAX_TARGET_OFFSET;
    }
    command = HeadingControl_ClampFloat(command, -zoneLimit, zoneLimit);

    s_targetOffset = HeadingControl_MoveToward(
        s_targetOffset, command, HEADING_CONTROL_OFFSET_SLEW_PER_TICK);
    if (HeadingControl_Abs(s_targetOffset) < 0.01f) {
        s_targetOffset = 0.0f;
    }

    SpeedControl_SetDifferentialTargetOffsetFloat(s_targetOffset);
}
