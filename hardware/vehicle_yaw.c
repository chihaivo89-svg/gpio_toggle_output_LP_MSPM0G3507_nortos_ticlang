#include "vehicle_yaw.h"

/* 5 ms sampling: 200 valid samples require one continuous stationary second. */
#define VEHICLE_YAW_CAL_SAMPLE_COUNT       (200U)
#define VEHICLE_YAW_STATIONARY_LIMIT_DPS   (5.0f)
#define VEHICLE_YAW_MDPS_PER_DPS           (1000.0f)
#define VEHICLE_YAW_SAMPLE_PERIOD_S         (0.005f)

/* The start reference uses the most recent continuous 300 ms at rest. */
#define VEHICLE_YAW_REFERENCE_SAMPLE_COUNT  (60U)
#define VEHICLE_YAW_REFERENCE_RATE_LIMIT_DPS (2.0f)

static bool s_enabled;
static bool s_calibrated;
static uint16_t s_calibrationSamples;
static float s_calibrationSumDps;
static float s_biasDps;
static float s_rateDps;
static float s_headingDeg;
static float s_referenceSamples[VEHICLE_YAW_REFERENCE_SAMPLE_COUNT];
static float s_referenceSumDeg;
static uint8_t s_referenceIndex;
static uint8_t s_referenceSampleCount;

static float VehicleYaw_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void VehicleYaw_ResetReferenceWindow(void)
{
    s_referenceSumDeg = 0.0f;
    s_referenceIndex = 0U;
    s_referenceSampleCount = 0U;
}

static void VehicleYaw_UpdateReferenceWindow(void)
{
    if (VehicleYaw_Abs(s_rateDps) >
        VEHICLE_YAW_REFERENCE_RATE_LIMIT_DPS) {
        VehicleYaw_ResetReferenceWindow();
        return;
    }

    if (s_referenceSampleCount < VEHICLE_YAW_REFERENCE_SAMPLE_COUNT) {
        s_referenceSamples[s_referenceSampleCount] = s_headingDeg;
        s_referenceSumDeg += s_headingDeg;
        s_referenceSampleCount++;
        if (s_referenceSampleCount == VEHICLE_YAW_REFERENCE_SAMPLE_COUNT) {
            s_referenceIndex = 0U;
        }
        return;
    }

    s_referenceSumDeg -= s_referenceSamples[s_referenceIndex];
    s_referenceSamples[s_referenceIndex] = s_headingDeg;
    s_referenceSumDeg += s_headingDeg;
    s_referenceIndex++;
    if (s_referenceIndex >= VEHICLE_YAW_REFERENCE_SAMPLE_COUNT) {
        s_referenceIndex = 0U;
    }
}

void VehicleYaw_Init(void)
{
    s_calibrated = false;
    s_calibrationSamples = 0U;
    s_calibrationSumDps = 0.0f;
    s_biasDps = 0.0f;
    s_rateDps = 0.0f;
    s_headingDeg = 0.0f;
    VehicleYaw_ResetReferenceWindow();

    /* Do not accept samples until IMU660RB_Init() has completed. */
    s_enabled = true;
}

void VehicleYaw_Update5ms(float gyroYMdps)
{
    float rawRateDps;

    if (!s_enabled) {
        return;
    }

    rawRateDps = gyroYMdps / VEHICLE_YAW_MDPS_PER_DPS;

    if (!s_calibrated) {
        /*
         * A visible turn restarts calibration instead of contaminating the
         * bias.  The car must stay still for one continuous second after boot.
         */
        if ((rawRateDps > -VEHICLE_YAW_STATIONARY_LIMIT_DPS) &&
            (rawRateDps < VEHICLE_YAW_STATIONARY_LIMIT_DPS)) {
            s_calibrationSumDps += rawRateDps;
            s_calibrationSamples++;

            if (s_calibrationSamples >= VEHICLE_YAW_CAL_SAMPLE_COUNT) {
                s_biasDps = s_calibrationSumDps /
                            (float)VEHICLE_YAW_CAL_SAMPLE_COUNT;
                s_calibrated = true;
                s_headingDeg = 0.0f;
                VehicleYaw_ResetReferenceWindow();
            }
        } else {
            s_calibrationSamples = 0U;
            s_calibrationSumDps = 0.0f;
        }

        s_rateDps = 0.0f;
        return;
    }

    /* Positive remains a vehicle-left turn, as verified on the real car. */
    s_rateDps = rawRateDps - s_biasDps;

    /*
     * This is only a vehicle-frame heading observation for validation.
     * No heading value is sent to Fusion or any motor-control interface.
     */
    s_headingDeg += s_rateDps * VEHICLE_YAW_SAMPLE_PERIOD_S;
    VehicleYaw_UpdateReferenceWindow();
}

bool VehicleYaw_IsCalibrated(void)
{
    return s_calibrated;
}

float VehicleYaw_GetRateDps(void)
{
    return s_rateDps;
}

float VehicleYaw_GetHeadingDeg(void)
{
    return s_headingDeg;
}

bool VehicleYaw_GetStationaryReferenceDeg(float *referenceDeg)
{
    if (referenceDeg == 0 ||
        s_referenceSampleCount < VEHICLE_YAW_REFERENCE_SAMPLE_COUNT) {
        return false;
    }

    *referenceDeg = s_referenceSumDeg /
                    (float)VEHICLE_YAW_REFERENCE_SAMPLE_COUNT;
    return true;
}
