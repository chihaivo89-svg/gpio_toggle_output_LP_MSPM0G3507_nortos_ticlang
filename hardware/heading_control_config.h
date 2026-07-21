#ifndef HEADING_CONTROL_CONFIG_H
#define HEADING_CONTROL_CONFIG_H

/*
 * 正式版车辆航向外环参数入口。
 *
 * The speed PI loop remains unchanged.  This controller only creates a small
 * differential target offset while heading hold is active.
 * Positive heading error means the vehicle has turned left of the saved
 * reference; the controller applies the opposite wheel-speed offset.
 */

#define HEADING_CONTROL_KP                         (0.45f)
#define HEADING_CONTROL_KD                         (0.06f)
#define HEADING_CONTROL_DEADBAND_DEG               (0.5f)
#define HEADING_CONTROL_MAX_TARGET_OFFSET          (3)

/*
 * Error and rate filtering keep the fractional target offset quiet without
 * delaying the 20 ms speed loop.  Zone hysteresis gives small errors gentle
 * authority and reserves +/-3 for a large disturbance.
 */
#define HEADING_CONTROL_ERROR_FILTER_ALPHA         (0.35f)
#define HEADING_CONTROL_YAW_RATE_FILTER_ALPHA      (0.20f)
#define HEADING_CONTROL_MEDIUM_ERROR_DEG           (1.5f)
#define HEADING_CONTROL_MEDIUM_RELEASE_DEG         (1.2f)
#define HEADING_CONTROL_STRONG_ERROR_DEG           (5.0f)
#define HEADING_CONTROL_STRONG_RELEASE_DEG         (4.0f)
#define HEADING_CONTROL_OFFSET_SLEW_PER_TICK       (0.50f)

#endif /* HEADING_CONTROL_CONFIG_H */
