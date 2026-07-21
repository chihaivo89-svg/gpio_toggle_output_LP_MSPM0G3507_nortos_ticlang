#ifndef HEADING_CONTROL_CONFIG_H
#define HEADING_CONTROL_CONFIG_H

/*
 * Temporary vehicle-heading outer-loop tuning defaults.
 *
 * The speed PI loop remains unchanged.  This controller only creates a small
 * differential target offset while a heading test is active.
 * Positive heading error means the vehicle has turned left of the saved
 * reference; the controller applies the opposite wheel-speed offset.
 */

#define HEADING_CONTROL_DEFAULT_KP                 (0.40f)
#define HEADING_CONTROL_DEFAULT_KD                 (0.00f)
#define HEADING_CONTROL_DEFAULT_DEADBAND_DEG       (0.8f)
#define HEADING_CONTROL_DEFAULT_MAX_TARGET_OFFSET  (1)

/*
 * The heading loop ultimately sends an integer target offset to the speed
 * loop.  Without filtering/hysteresis, small IMU and floor disturbances can
 * make that integer jump between -1, 0 and +1 many times per second.
 */
#define HEADING_CONTROL_ERROR_FILTER_ALPHA         (0.35f)
#define HEADING_CONTROL_OFFSET_RELEASE_DEG         (0.25f)
#define HEADING_CONTROL_OFFSET_HOLD_TICKS          (3)

/* UART tuning guardrails.  They intentionally keep this test outer loop mild. */
#define HEADING_CONTROL_KP_MIN                     (0.0f)
#define HEADING_CONTROL_KP_MAX                     (1.00f)
#define HEADING_CONTROL_KD_MIN                     (0.0f)
#define HEADING_CONTROL_KD_MAX                     (0.20f)
#define HEADING_CONTROL_DEADBAND_MIN_DEG           (0.0f)
#define HEADING_CONTROL_DEADBAND_MAX_DEG           (5.0f)
#define HEADING_CONTROL_MAX_TARGET_OFFSET_LIMIT    (1)

#endif /* HEADING_CONTROL_CONFIG_H */
