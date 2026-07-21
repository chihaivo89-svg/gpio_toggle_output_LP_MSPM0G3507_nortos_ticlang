#ifndef VEHICLE_YAW_H
#define VEHICLE_YAW_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Vehicle-frame yaw-rate helper for the vertically mounted IMU660RB.
 * It is independent from the existing Fusion attitude solution.
 */
void VehicleYaw_Init(void);
void VehicleYaw_Update5ms(float gyroYMdps);

bool VehicleYaw_IsCalibrated(void);
uint16_t VehicleYaw_GetCalibrationSamples(void);
float VehicleYaw_GetBiasDps(void);
float VehicleYaw_GetRateDps(void);
float VehicleYaw_GetHeadingDeg(void);

#endif /* VEHICLE_YAW_H */
