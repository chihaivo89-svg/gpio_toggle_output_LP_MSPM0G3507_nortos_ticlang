#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    float referenceDeg;
    float headingDeg;
    float errorDeg;
    float yawRateDps;
    int32_t targetOffset;
    float kp;
    float kd;
    float deadbandDeg;
    int32_t maxTargetOffset;
} HeadingControlTelemetry;

void HeadingControl_Init(void);

/* Starts an independent vehicle-heading hold at the current heading. */
bool HeadingControl_Start(void);

/* Stops heading correction and immediately removes any speed target offset. */
void HeadingControl_Stop(void);

bool HeadingControl_IsActive(void);

/* Call once per 20 ms speed-control sample before SpeedControl_Update20ms(). */
void HeadingControl_Update20ms(void);

bool HeadingControl_SetKp(float value);
bool HeadingControl_SetKd(float value);
bool HeadingControl_SetDeadbandDeg(float value);
bool HeadingControl_SetMaxTargetOffset(int32_t value);

void HeadingControl_GetTelemetry(HeadingControlTelemetry *telemetry);

#endif /* HEADING_CONTROL_H */
