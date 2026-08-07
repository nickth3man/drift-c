#include "dev/dev_scenario.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "core/config.h"

/* Scenario scripts are written in seconds and converted here, so a change to FIXED_HZ moves
 * the tick boundaries without rewriting a single scenario. */
#define AT(seconds) ((uint64_t)((seconds) * (double)FIXED_HZ))

typedef enum {
    SCENARIO_FREE = 0,
    SCENARIO_ACCEL,
    SCENARIO_SKIDPAD,
    SCENARIO_STEP_STEER,
    SCENARIO_LIFT_OFF,
    SCENARIO_POWER_OVERSTEER,
    SCENARIO_HANDBRAKE_ENTRY,
    SCENARIO_TRANSITION,
    SCENARIO_BRAKE_CORNER,
    SCENARIO_COAST_DOWN,
    SCENARIO_ACCEL_LOAD,
    SCENARIO_BRAKE_LOAD,
    SCENARIO_CATCHABLE_DRIFT,
    SCENARIO_SINE_STEER,
    SCENARIO_SINE_DWELL,
    SCENARIO_J_TURN,
    SCENARIO_DOUBLE_LANE_CHANGE,
    SCENARIO_SLALOM,
    SCENARIO_FISHHOOK_RECOVERY,
    SCENARIO_EMERGENCY_OBSTACLE_LEFT,
    SCENARIO_EMERGENCY_OBSTACLE_RIGHT,
    SCENARIO_BRAKE_TURN_LEFT,
    SCENARIO_BRAKE_TURN_RIGHT,
    SCENARIO_THROTTLE_STEP,
    SCENARIO_BRAKE_STEP,
    SCENARIO_STEERING_RAMP,
    SCENARIO_STEERING_PULSE,
    SCENARIO_THROTTLE_PULSE,
    SCENARIO_LIFT_OFF_LEFT,
    SCENARIO_LIFT_OFF_RIGHT,
    SCENARIO_POWER_ON_LEFT,
    SCENARIO_POWER_ON_RIGHT,
    SCENARIO_TRAIL_BRAKE_LEFT,
    SCENARIO_TRAIL_BRAKE_RIGHT,
    SCENARIO_CONSTANT_RADIUS_LEFT,
    SCENARIO_CONSTANT_RADIUS_RIGHT,
    SCENARIO_FIGURE_EIGHT,
    SCENARIO_CHICANE,
    SCENARIO_LOW_SPEED_TIGHT_TURN_LEFT,
    SCENARIO_LOW_SPEED_TIGHT_TURN_RIGHT,
    SCENARIO_STOP_AND_GO,
    SCENARIO_GEAR_SHIFT_ACCEL,
    SCENARIO_COAST_BRAKE_PULSE
} ScenarioIndex;

static const DevScenario g_scenarios[] = {
    { "free", "no scripted input; drive it yourself", 0, 0u },
    { "accel", "standing start at full throttle, straight ahead", 1200, 1001u },
    { "skidpad", "constant radius: settle, then hold steady steer and throttle", 2400, 1002u },
    { "step-steer", "build speed, step the steering, hold it, then return to centre", 960,
      1003u },
    { "lift-off", "steady cornering interrupted by a throttle lift", 1200, 1004u },
    { "power-oversteer", "steady steer, then full throttle until the rear breaks", 1200,
      1005u },
    { "handbrake-entry", "handbrake pull into a corner, release, then counter-steer", 1200,
      1006u },
    { "transition", "left/right transitions at a constant period", 1800, 1007u },
    { "brake-corner", "braking while already turning", 1200, 1008u },
    { "coast-down", "accelerate, then coast with every control released", 2400, 1009u },
    { "accel-load", "straight full-throttle launch: load transfers rearward", 720, 1010u },
    { "brake-load", "accelerate, then brake to a stop: load transfers forward", 1080, 1011u },
    { "catchable-drift", "initiate, hold, countersteer, and recover a slide", 1200, 1012u },
    { "sine-steer", "sinusoidal steering sweep for nonlinear lateral response", 1200, 2001u },
    { "sine-dwell", "sine-with-dwell steering reversal and recovery", 1440, 2002u },
    { "j-turn", "throttle lift followed by a single aggressive steering turn", 900, 2003u },
    { "double-lane-change", "left-right-left evasive lane-change sequence", 1200, 2004u },
    { "slalom", "repeated alternating steering through a slalom", 1800, 2005u },
    { "fishhook-recovery", "ramp steer, hold, mirror steer, and recover", 1200, 2006u },
    { "emergency-obstacle-left", "brake and steer left-right around an obstacle", 1200, 2007u },
    { "emergency-obstacle-right", "brake and steer right-left around an obstacle", 1200,
      2008u },
    { "brake-turn-left", "combined braking and left cornering at the limit", 1200, 2009u },
    { "brake-turn-right", "combined braking and right cornering at the limit", 1200, 2010u },
    { "throttle-step", "mid-corner throttle step and transient response", 900, 2011u },
    { "brake-step", "mid-corner service-brake step and load transfer", 900, 2012u },
    { "steering-ramp", "linear steering ramp into a sustained corner", 900, 2013u },
    { "steering-pulse", "short steering pulse followed by straight recovery", 900, 2014u },
    { "throttle-pulse", "short full-throttle pulse during cornering", 900, 2015u },
    { "lift-off-left", "left corner throttle lift and yaw recovery", 1200, 2016u },
    { "lift-off-right", "right corner throttle lift and yaw recovery", 1200, 2017u },
    { "power-on-left", "left corner power application and rear saturation", 1200, 2018u },
    { "power-on-right", "right corner power application and rear saturation", 1200, 2019u },
    { "trail-brake-left", "left corner entry with brake release while turning", 1200, 2020u },
    { "trail-brake-right", "right corner entry with brake release while turning", 1200, 2021u },
    { "constant-radius-left", "steady left-radius corner with settled lateral load", 1800,
      2022u },
    { "constant-radius-right", "steady right-radius corner with settled lateral load", 1800,
      2023u },
    { "figure-eight", "opposite-hand corner lobes with a controlled reversal", 2400, 2024u },
    { "chicane", "rapid left-right chicane with a straight exit", 1500, 2025u },
    { "low-speed-tight-turn-left", "low-speed full-lock left kinematic blend", 900, 2026u },
    { "low-speed-tight-turn-right", "low-speed full-lock right kinematic blend", 900, 2027u },
    { "stop-and-go", "two launch and stop cycles with full braking", 1500, 2028u },
    { "gear-shift-accel", "full-throttle acceleration through automatic shifts", 1800, 2029u },
    { "coast-brake-pulse", "acceleration, coast, brake pulse, and release", 1500, 2030u },
};

#define SCENARIO_COUNT ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

int dev_scenario_count(void)
{
    return SCENARIO_COUNT;
}

const DevScenario *dev_scenario_at(int index)
{
    if (index < 0 || index >= SCENARIO_COUNT) return NULL;
    return &g_scenarios[index];
}

int dev_scenario_find(const char *name)
{
    if (name == NULL) return -1;
    for (int i = 0; i < SCENARIO_COUNT; i++) {
        if (strcmp(g_scenarios[i].name, name) == 0) return i;
    }
    return -1;
}

static float sine_input(uint64_t tick, float startSeconds, float periodSeconds, float amplitude)
{
    const float timeS = (float)tick / (float)FIXED_HZ;
    if (timeS < startSeconds) return 0.0f;
    const float phase = (timeS - startSeconds) * 2.0f * 3.14159265359f / periodSeconds;
    return amplitude * sinf(phase);
}

static float linear_ramp(uint64_t tick, float startSeconds, float durationSeconds, float end)
{
    const float timeS = (float)tick / (float)FIXED_HZ;
    if (timeS <= startSeconds) return 0.0f;
    if (timeS >= startSeconds + durationSeconds) return end;
    return end * (timeS - startSeconds) / durationSeconds;
}

static float alternating_steer(uint64_t tick, float startSeconds, float periodSeconds,
                               float amplitude)
{
    const float timeS = (float)tick / (float)FIXED_HZ;
    if (timeS < startSeconds) return 0.0f;
    const uint64_t phase = (uint64_t)((timeS - startSeconds) / periodSeconds);
    return (phase % 2u == 0u) ? amplitude : -amplitude;
}

void dev_scenario_input(int index, uint64_t tick, Input *out)
{
    if (out == NULL) return;
    input_zero(out);
    if (index <= SCENARIO_FREE || index >= SCENARIO_COUNT) return;

    switch (index) {
        case SCENARIO_ACCEL: out->throttle = 1.0f; break;

        case SCENARIO_SKIDPAD:
            /* Two seconds of straight-line acceleration, then steering wound on over a
             * second and held with partial throttle for the remaining seventeen. The ramp
             * matters: stepping to full lock would measure a transient, and this scenario
             * exists to measure a steady state. Steady state is whatever the model settles
             * on — the script states no target. */
            if (tick < AT(2.0)) {
                out->throttle = 1.0f;
            } else {
                const uint64_t sinceEntry = tick - AT(2.0);
                const float ramp =
                    (sinceEntry >= AT(1.0)) ? 1.0f : (float)sinceEntry / (float)AT(1.0);
                out->steer = 0.25f * ramp;
                out->throttle = 0.30f;
            }
            break;

        case SCENARIO_STEP_STEER:
            /* Three seconds to reach a settled speed, a step held for three and a half, then
             * back to centre so the return-to-straight half of the response is measurable
             * too. Rise time, overshoot, and settling are read off the hold; recovery is
             * read off the release. */
            out->throttle = (tick < AT(3.0)) ? 0.60f : 0.30f;
            if (tick >= AT(3.0) && tick < AT(6.5)) out->steer = 0.20f;
            break;

        case SCENARIO_LIFT_OFF:
            /* The corner has to be STABLE before the lift, or the transient is unreadable:
             * a car already past the rear tires' peak has nowhere further to rotate. */
            out->steer = (tick < AT(2.0)) ? 0.0f : 0.22f;
            out->throttle = (tick < AT(6.0)) ? 0.45f : 0.0f;
            break;

        case SCENARIO_POWER_OVERSTEER:
            out->steer = 0.40f;
            out->throttle = (tick < AT(2.0)) ? 0.40f : 1.0f;
            break;

        case SCENARIO_HANDBRAKE_ENTRY:
            out->throttle = (tick < AT(3.0)) ? 1.0f : 0.0f;
            if (tick >= AT(3.0) && tick < AT(3.75)) {
                out->steer = 0.50f;
                out->handbrake = 1.0f;
            } else if (tick >= AT(3.75) && tick < AT(6.0)) {
                out->steer = -0.30f; /* counter-steer: right is negative */
                out->throttle = 0.35f;
            }
            break;

        case SCENARIO_TRANSITION: {
            out->throttle = (tick < AT(2.0)) ? 0.80f : 0.35f;
            if (tick >= AT(2.0)) {
                /* 1.5 s per half-period, alternating sign. */
                const uint64_t phase = (tick - AT(2.0)) / AT(1.5);
                out->steer = ((phase % 2u) == 0u) ? 0.35f : -0.35f;
            }
            break;
        }

        case SCENARIO_BRAKE_CORNER:
            out->throttle = (tick < AT(4.0)) ? 1.0f : 0.0f;
            if (tick >= AT(4.0)) {
                out->steer = 0.30f;
                out->brake = 0.80f;
            }
            break;

        case SCENARIO_COAST_DOWN: out->throttle = (tick < AT(6.0)) ? 1.0f : 0.0f; break;

        case SCENARIO_ACCEL_LOAD:
            /* Five seconds of straight full throttle, then one of coast. No steering at all:
             * the only thing that may move the axle loads is longitudinal acceleration. */
            out->throttle = (tick < AT(5.0)) ? 1.0f : 0.0f;
            break;

        case SCENARIO_BRAKE_LOAD:
            /* Four seconds of acceleration, then full service braking to a standstill. */
            out->throttle = (tick < AT(4.0)) ? 1.0f : 0.0f;
            out->brake = (tick >= AT(4.0)) ? 1.0f : 0.0f;
            break;

        case SCENARIO_CATCHABLE_DRIFT:
            /* Five stages, all through ordinary controls: build speed, break the rear loose
             * with the handbrake, hold the slide on throttle, catch it on countersteer, then
             * unwind to straight travel. Nothing here reaches into the physics. */
            if (tick < AT(2.5)) { /* 1. build speed */
                out->throttle = 1.0f;
            } else if (tick < AT(3.2)) { /* 2. initiate */
                out->steer = 0.60f;
                out->handbrake = 1.0f;
            } else if (tick < AT(4.6)) { /* 3. hold the slide */
                out->steer = 0.25f;
                out->throttle = 0.55f;
            } else if (tick < AT(6.6)) { /* 4. countersteer */
                out->steer = -0.55f;
                out->throttle = 0.30f;
            } else { /* 5. recover */
                out->steer = 0.0f;
                out->throttle = 0.25f;
            }
            break;

        case SCENARIO_SINE_STEER:
            out->throttle = (tick < AT(2.0)) ? 1.0f : 0.35f;
            out->steer = sine_input(tick, 2.0f, 4.0f, 0.22f);
            break;
        case SCENARIO_SINE_DWELL:
            out->steer = 0.0f;
            out->throttle = (tick < AT(2.0)) ? 1.0f : 0.35f;
            if (tick >= AT(2.0) && tick < AT(3.0))
                out->steer = linear_ramp(tick, 2.0f, 1.0f, 0.25f);
            else if (tick >= AT(3.0) && tick < AT(5.0))
                out->steer = 0.25f;
            else if (tick >= AT(5.0) && tick < AT(6.0))
                out->steer = linear_ramp(tick, 5.0f, 1.0f, -0.25f);
            else if (tick >= AT(6.0) && tick < AT(9.0))
                out->steer = -0.25f;
            else if (tick >= AT(9.0))
                out->steer = linear_ramp(tick, 9.0f, 1.0f, 0.0f);
            break;
        case SCENARIO_J_TURN:
            out->throttle = (tick < AT(3.0)) ? 0.80f : 0.0f;
            out->steer = linear_ramp(tick, 3.0f, 1.0f, 0.45f);
            break;
        case SCENARIO_DOUBLE_LANE_CHANGE:
            out->steer = 0.0f;
            out->throttle = (tick < AT(3.0)) ? 0.60f : 0.45f;
            if (tick >= AT(3.0) && tick < AT(4.0))
                out->steer = 0.35f;
            else if (tick >= AT(4.0) && tick < AT(5.5))
                out->steer = -0.35f;
            else if (tick >= AT(5.5) && tick < AT(6.5))
                out->steer = 0.35f;
            break;
        case SCENARIO_SLALOM:
            out->throttle = (tick < AT(3.0)) ? 0.60f : 0.55f;
            out->steer = alternating_steer(tick, 3.0f, 1.0f, 0.30f);
            break;
        case SCENARIO_FISHHOOK_RECOVERY:
            out->steer = 0.0f;
            out->throttle = (tick < AT(3.0)) ? 0.55f : 0.35f;
            if (tick >= AT(3.0) && tick < AT(4.0))
                out->steer = linear_ramp(tick, 3.0f, 1.0f, 0.55f);
            else if (tick >= AT(4.0) && tick < AT(5.0))
                out->steer = 0.55f;
            else if (tick >= AT(5.0) && tick < AT(6.0))
                out->steer = linear_ramp(tick, 5.0f, 1.0f, -0.55f);
            else if (tick >= AT(6.0) && tick < AT(7.0))
                out->steer = -0.55f;
            break;
        case SCENARIO_EMERGENCY_OBSTACLE_LEFT:
            out->steer = 0.0f;
            out->throttle = (tick < AT(3.0)) ? 0.80f : 0.0f;
            out->brake = (tick >= AT(3.0)) ? 0.80f : 0.0f;
            if (tick >= AT(3.0) && tick < AT(4.0))
                out->steer = 0.40f;
            else if (tick >= AT(4.0) && tick < AT(5.0))
                out->steer = -0.40f;
            break;
        case SCENARIO_EMERGENCY_OBSTACLE_RIGHT:
            out->steer = 0.0f;
            out->throttle = (tick < AT(3.0)) ? 0.80f : 0.0f;
            out->brake = (tick >= AT(3.0)) ? 0.80f : 0.0f;
            if (tick >= AT(3.0) && tick < AT(4.0))
                out->steer = -0.40f;
            else if (tick >= AT(4.0) && tick < AT(5.0))
                out->steer = 0.40f;
            break;
        case SCENARIO_BRAKE_TURN_LEFT:
            out->throttle = (tick < AT(3.0)) ? 0.80f : 0.0f;
            out->brake = (tick >= AT(3.0)) ? 0.75f : 0.0f;
            out->steer = (tick >= AT(3.0)) ? 0.28f : 0.0f;
            break;
        case SCENARIO_BRAKE_TURN_RIGHT:
            out->throttle = (tick < AT(3.0)) ? 0.80f : 0.0f;
            out->brake = (tick >= AT(3.0)) ? 0.75f : 0.0f;
            out->steer = (tick >= AT(3.0)) ? -0.28f : 0.0f;
            break;
        case SCENARIO_THROTTLE_STEP:
            out->steer = (tick >= AT(2.0)) ? 0.22f : 0.0f;
            out->throttle = (tick < AT(3.5)) ? 0.20f : 0.85f;
            break;
        case SCENARIO_BRAKE_STEP:
            out->throttle = (tick < AT(3.0)) ? 0.70f : 0.0f;
            out->steer = (tick >= AT(2.0)) ? 0.22f : 0.0f;
            out->brake = (tick >= AT(3.0)) ? 0.70f : 0.0f;
            break;
        case SCENARIO_STEERING_RAMP:
            out->throttle = 0.60f;
            out->steer = linear_ramp(tick, 2.0f, 4.0f, 0.40f);
            break;
        case SCENARIO_STEERING_PULSE:
            out->throttle = 0.45f;
            out->steer = 0.0f;
            if (tick >= AT(3.0) && tick < AT(4.0)) out->steer = 0.35f;
            break;
        case SCENARIO_THROTTLE_PULSE:
            out->steer = (tick >= AT(2.0)) ? 0.22f : 0.0f;
            if (tick < AT(3.0))
                out->throttle = 0.25f;
            else if (tick < AT(4.0))
                out->throttle = 1.0f;
            else
                out->throttle = 0.25f;
            break;
        case SCENARIO_LIFT_OFF_LEFT:
            out->steer = 0.25f;
            out->throttle = (tick < AT(5.0)) ? 0.55f : 0.0f;
            break;
        case SCENARIO_LIFT_OFF_RIGHT:
            out->steer = -0.25f;
            out->throttle = (tick < AT(5.0)) ? 0.55f : 0.0f;
            break;
        case SCENARIO_POWER_ON_LEFT:
            out->steer = 0.30f;
            out->throttle = (tick < AT(3.0)) ? 0.20f : 1.0f;
            break;
        case SCENARIO_POWER_ON_RIGHT:
            out->steer = -0.30f;
            out->throttle = (tick < AT(3.0)) ? 0.20f : 1.0f;
            break;
        case SCENARIO_TRAIL_BRAKE_LEFT:
            out->steer = 0.30f;
            if (tick < AT(3.0))
                out->throttle = 1.0f;
            else if (tick < AT(5.0))
                out->brake = linear_ramp(tick, 3.0f, 2.0f, 0.80f);
            break;
        case SCENARIO_TRAIL_BRAKE_RIGHT:
            out->steer = -0.30f;
            if (tick < AT(3.0))
                out->throttle = 1.0f;
            else if (tick < AT(5.0))
                out->brake = linear_ramp(tick, 3.0f, 2.0f, 0.80f);
            break;
        case SCENARIO_CONSTANT_RADIUS_LEFT:
            out->throttle = (tick < AT(2.0)) ? 0.80f : 0.55f;
            out->steer = linear_ramp(tick, 2.0f, 1.0f, 0.25f);
            break;
        case SCENARIO_CONSTANT_RADIUS_RIGHT:
            out->throttle = (tick < AT(2.0)) ? 0.80f : 0.55f;
            out->steer = linear_ramp(tick, 2.0f, 1.0f, -0.25f);
            break;
        case SCENARIO_FIGURE_EIGHT:
            out->throttle = (tick < AT(2.0)) ? 0.80f : 0.55f;
            out->steer = alternating_steer(tick, 2.0f, 4.0f, 0.28f);
            break;
        case SCENARIO_CHICANE:
            out->steer = 0.0f;
            out->throttle = (tick < AT(3.0)) ? 0.60f : 0.30f;
            if (tick >= AT(3.0) && tick < AT(4.0))
                out->steer = 0.35f;
            else if (tick >= AT(4.0) && tick < AT(5.0))
                out->steer = -0.35f;
            break;
        case SCENARIO_LOW_SPEED_TIGHT_TURN_LEFT:
            out->throttle = 0.15f;
            out->steer = (tick >= AT(2.0)) ? 0.45f : 0.0f;
            break;
        case SCENARIO_LOW_SPEED_TIGHT_TURN_RIGHT:
            out->throttle = 0.15f;
            out->steer = (tick >= AT(2.0)) ? -0.45f : 0.0f;
            break;
        case SCENARIO_STOP_AND_GO:
            if (tick < AT(3.0) || (tick >= AT(4.0) && tick < AT(7.0))) out->throttle = 1.0f;
            if ((tick >= AT(3.0) && tick < AT(4.0)) || tick >= AT(7.0)) out->brake = 1.0f;
            break;
        case SCENARIO_GEAR_SHIFT_ACCEL: out->throttle = 1.0f; break;
        case SCENARIO_COAST_BRAKE_PULSE:
            if (tick < AT(3.0))
                out->throttle = 1.0f;
            else if (tick >= AT(4.0) && tick < AT(5.0))
                out->brake = 0.70f;
            break;
        default: break;
    }
}
