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
float VehicleYaw_GetRateDps(void);
float VehicleYaw_GetHeadingDeg(void);

/* Returns the average heading only after 300 ms of continuous standstill. */
bool VehicleYaw_GetStationaryReferenceDeg(float *referenceDeg);

#endif /* VEHICLE_YAW_H */
