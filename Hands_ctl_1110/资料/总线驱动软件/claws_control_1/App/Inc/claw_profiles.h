#ifndef CLAW_PROFILES_H
#define CLAW_PROFILES_H
#include "claw_pid.h"

/* Select ONE physical claw, then rebuild/flash. No extra CAN nodes/channels.
 * NEW MS4931 board: all 100-permille values are placeholders, NOT validated.
 * Mask=7 enables modules 4..6; module 5 PI trial is not hardware validated.
 * Never copy the old driver's 618-permille input as an equivalent motor torque.
 * All rows are UNCALIBRATED initial values, including modules 1..3.
 * Fields: Kp Ki Kd I_limit min max startup confirm_counts timeout_ms. */
#ifndef CLAW_PROFILE_GROUP
#define CLAW_PROFILE_GROUP 1
#endif
#if CLAW_PROFILE_GROUP < 1 || CLAW_PROFILE_GROUP > 4
#error "CLAW_PROFILE_GROUP must be 1..4"
#endif
static const ClawPidConfig claw_module_profiles[12] = {
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  1 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  2 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  3 */
  {12, 0, 0, 150, 100, 1000, 600, 2, 200}, /*  4: full PWM range, 60% startup; initial P only */
  {12, 20, 0, 150, 100, 1000, 600, 2, 200}, /*  5: PI trial for persistent endpoint error */
  {12, 0, 0, 150, 100, 1000, 600, 2, 200}, /*  6: full PWM range, 60% startup; initial P only */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  7 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  8 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /*  9 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /* 10 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}, /* 11 */
  {12, 0, 0, 150, 100, 100, 100, 2, 400}  /* 12 */
};
#endif
