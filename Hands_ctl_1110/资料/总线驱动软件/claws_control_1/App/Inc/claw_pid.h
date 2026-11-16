#ifndef CLAW_PID_H
#define CLAW_PID_H

/* Pure controller shared by firmware and native offline tests.
 * Error: directed Hall counts behind a common time trajectory.
 * Output: PWM permille, NOT current or torque. */
typedef struct {
  float kp, ki, kd; /* permille/count, permille/(count*s), permille*s/count */
  float integral_limit;
  unsigned short minimum, maximum, startup;
  unsigned short startup_counts, startup_ms;
} ClawPidConfig;

typedef struct {
  float integral, previous_progress, velocity;
} ClawPidState;

static void ClawPid_Reset(ClawPidState *s, float progress)
{
  s->integral = 0.0f;
  s->previous_progress = progress;
  s->velocity = 0.0f;
}

static unsigned short ClawPid_Step(ClawPidState *s, const ClawPidConfig *p,
                                  float reference, float progress, float dt,
                                  unsigned short ceiling)
{
  float error = reference - progress;
  float candidate, output;
  if (ceiling > p->maximum) ceiling = p->maximum;
  /* Derivative on measured progress; 50 ms low pass suppresses Hall steps. */
  s->velocity += dt / (0.05f + dt) *
      ((progress - s->previous_progress) / dt - s->velocity);
  s->previous_progress = progress;
  candidate = s->integral + p->ki * error * dt;
  if (candidate > p->integral_limit) candidate = p->integral_limit;
  if (candidate < -p->integral_limit) candidate = -p->integral_limit;
  output = p->kp * error + candidate - p->kd * s->velocity;
  /* Conditional integration at both output limits. */
  if (!((output > ceiling && error > 0.0f) ||
        (output < p->minimum && error < 0.0f))) s->integral = candidate;
  output = p->kp * error + s->integral - p->kd * s->velocity;
  /* Ahead of schedule: coast; never reverse to chase the trajectory. */
  if (error <= 0.0f) return 0;
  if (output < p->minimum) output = (float)p->minimum;
  if (output > ceiling) output = (float)ceiling;
  return (unsigned short)(output + 0.5f);
}
#endif
