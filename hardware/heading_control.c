#include "heading_control.h"

#include "heading_control_config.h"
#include "speed_control.h"
#include "vehicle_yaw.h"

static HeadingControlTelemetry s_telemetry;
static float s_filteredErrorDeg;
static uint8_t s_offsetHoldTicks;

static float HeadingControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t HeadingControl_ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static int32_t HeadingControl_RoundToInt32(float value)
{
    return (value >= 0.0f) ?
        (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static void HeadingControl_UpdateObservation(void)
{
    s_telemetry.headingDeg = VehicleYaw_GetHeadingDeg();
    s_telemetry.yawRateDps = VehicleYaw_GetRateDps();
    s_telemetry.errorDeg = s_telemetry.headingDeg -
                           s_telemetry.referenceDeg;
}

void HeadingControl_Init(void)
{
    s_telemetry.active = false;
    s_telemetry.referenceDeg = 0.0f;
    s_telemetry.headingDeg = 0.0f;
    s_telemetry.errorDeg = 0.0f;
    s_telemetry.yawRateDps = 0.0f;
    s_telemetry.targetOffset = 0;
    s_telemetry.kp = HEADING_CONTROL_DEFAULT_KP;
    s_telemetry.kd = HEADING_CONTROL_DEFAULT_KD;
    s_telemetry.deadbandDeg = HEADING_CONTROL_DEFAULT_DEADBAND_DEG;
    s_telemetry.maxTargetOffset =
        HEADING_CONTROL_DEFAULT_MAX_TARGET_OFFSET;
    s_filteredErrorDeg = 0.0f;
    s_offsetHoldTicks = 0U;

    SpeedControl_SetDifferentialTargetOffset(0);
}

bool HeadingControl_Start(void)
{
    if (!VehicleYaw_IsCalibrated() || !SpeedControl_IsRunning()) {
        return false;
    }

    HeadingControl_UpdateObservation();
    s_telemetry.referenceDeg = s_telemetry.headingDeg;
    s_telemetry.errorDeg = 0.0f;
    s_telemetry.targetOffset = 0;
    s_filteredErrorDeg = 0.0f;
    s_offsetHoldTicks = 0U;
    s_telemetry.active = true;
    SpeedControl_SetDifferentialTargetOffset(0);
    return true;
}

void HeadingControl_Stop(void)
{
    s_telemetry.active = false;
    s_telemetry.targetOffset = 0;
    s_filteredErrorDeg = 0.0f;
    s_offsetHoldTicks = 0U;
    SpeedControl_SetDifferentialTargetOffset(0);
}

bool HeadingControl_IsActive(void)
{
    return s_telemetry.active;
}

void HeadingControl_Update20ms(void)
{
    float proportionalError;
    float absoluteError;
    float command;
    int32_t roundedCommand;

    HeadingControl_UpdateObservation();

    if (!s_telemetry.active) {
        return;
    }

    if (!VehicleYaw_IsCalibrated() || !SpeedControl_IsRunning()) {
        HeadingControl_Stop();
        return;
    }

    /*
     * Keep the raw heading telemetry visible, but drive the integer target
     * offset from a lightly filtered error.  This removes the rapid -1/0/+1
     * chattering seen on road tests without adding visible steering lag.
     */
    s_filteredErrorDeg +=
        (s_telemetry.errorDeg - s_filteredErrorDeg) *
        HEADING_CONTROL_ERROR_FILTER_ALPHA;

    proportionalError = s_filteredErrorDeg;
    absoluteError = HeadingControl_Abs(proportionalError);
    if (HeadingControl_Abs(proportionalError) <=
        s_telemetry.deadbandDeg) {
        proportionalError = 0.0f;
    } else if (proportionalError > 0.0f) {
        proportionalError -= s_telemetry.deadbandDeg;
    } else {
        proportionalError += s_telemetry.deadbandDeg;
    }

    /*
     * Positive heading error means the vehicle has turned left of the saved
     * reference.  The tested vehicle wiring needs the opposite target offset
     * so the chassis steers back toward the reference instead of running away.
     */
    command = s_telemetry.kp * proportionalError +
              s_telemetry.kd * s_telemetry.yawRateDps;
    command = -command;
    roundedCommand = HeadingControl_RoundToInt32(command);

    /*
     * Target offset is an integer.  Do not immediately force every tiny
     * post-deadband error to +/-1; that made the vehicle weave.  Keep a small
     * nonzero correction briefly, release it only after the filtered error is
     * clearly near zero, and let larger errors still reach +/-2 normally.
     */
    if (roundedCommand == 0 && command != 0.0f &&
        s_telemetry.maxTargetOffset > 0 &&
        absoluteError >= (s_telemetry.deadbandDeg +
                          HEADING_CONTROL_OFFSET_RELEASE_DEG)) {
        roundedCommand = (command > 0.0f) ? 1 : -1;
    }

    roundedCommand = HeadingControl_ClampInt32(
        roundedCommand,
        -s_telemetry.maxTargetOffset,
        s_telemetry.maxTargetOffset);

    if (roundedCommand != 0) {
        s_telemetry.targetOffset = roundedCommand;
        s_offsetHoldTicks = HEADING_CONTROL_OFFSET_HOLD_TICKS;
    } else if (s_telemetry.targetOffset != 0 &&
               absoluteError > HEADING_CONTROL_OFFSET_RELEASE_DEG &&
               s_offsetHoldTicks > 0U) {
        s_offsetHoldTicks--;
    } else {
        s_telemetry.targetOffset = 0;
        s_offsetHoldTicks = 0U;
    }

    SpeedControl_SetDifferentialTargetOffset(s_telemetry.targetOffset);
}

bool HeadingControl_SetKp(float value)
{
    if (value < HEADING_CONTROL_KP_MIN || value > HEADING_CONTROL_KP_MAX) {
        return false;
    }

    s_telemetry.kp = value;
    return true;
}

bool HeadingControl_SetKd(float value)
{
    if (value < HEADING_CONTROL_KD_MIN || value > HEADING_CONTROL_KD_MAX) {
        return false;
    }

    s_telemetry.kd = value;
    return true;
}

bool HeadingControl_SetDeadbandDeg(float value)
{
    if (value < HEADING_CONTROL_DEADBAND_MIN_DEG ||
        value > HEADING_CONTROL_DEADBAND_MAX_DEG) {
        return false;
    }

    s_telemetry.deadbandDeg = value;
    return true;
}

bool HeadingControl_SetMaxTargetOffset(int32_t value)
{
    if (value < 0 || value > HEADING_CONTROL_MAX_TARGET_OFFSET_LIMIT) {
        return false;
    }

    s_telemetry.maxTargetOffset = value;
    if (s_telemetry.targetOffset > value ||
        s_telemetry.targetOffset < -value) {
        s_telemetry.targetOffset = HeadingControl_ClampInt32(
            s_telemetry.targetOffset, -value, value);
        SpeedControl_SetDifferentialTargetOffset(s_telemetry.targetOffset);
    }
    return true;
}

void HeadingControl_GetTelemetry(HeadingControlTelemetry *telemetry)
{
    if (telemetry != 0) {
        *telemetry = s_telemetry;
    }
}
