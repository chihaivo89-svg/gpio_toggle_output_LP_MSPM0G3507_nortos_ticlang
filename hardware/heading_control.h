#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>

void HeadingControl_Init(void);

/* Starts heading hold using a stationary reference captured before motor start. */
bool HeadingControl_Start(float referenceDeg);

/* Stops heading correction and immediately removes any speed target offset. */
void HeadingControl_Stop(void);

/* Call once per 20 ms speed-control sample before SpeedControl_Update20ms(). */
void HeadingControl_Update20ms(void);

#endif /* HEADING_CONTROL_H */
