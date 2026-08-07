/*
 * config.h — simulation, numerical infrastructure, and presentation constants.
 *
 * Rules enforced by this file:
 *
 *   - All physical values are SI. Every unit-bearing constant states its unit in its name
 *     or in a trailing comment. Constants with no unit are dimensionless ratios and say so.
 *   - PIXELS_PER_METER is a render scale. It is consumed only by units.h helpers and by
 *     rendering code. No simulation quantity may be derived from it. This is a regression
 *     test (tests/scenarios/core_tests.c, scenario "renderscale").
 *
 * | Quantity | Unit | Suffix convention |
 * |----------|------|-------------------|
 * | Position, length, CG height | meters | `...M` |
 * | Linear velocity | meters/second | `...Mps` |
 * | Linear acceleration | meters/second² | `...Mps2` |
 * | Mass | kilograms | `...Kg` |
 * | Force | newtons | `...N` |
 * | Torque | newton-meters | `...Nm` |
 * | Yaw inertia | kg·m² | `...KgM2` |
 * | Angle (heading, steering, slip) | radians | `...Rad` |
 * | Angular velocity (yaw, wheel) | radians/second | `...RadS` |
 * | Time | seconds | `...S` |
 *
 * Every physics constant carries its unit in its name or in a trailing comment. Constants
 * without units are dimensionless ratios and must be documented as such.
 *
 * Phase 2 added nonlinear tires, rear-wheel drive, wheel rotation, physical brakes, and
 * combined slip. Phase 3 adds longitudinal load transfer from a filtered previous-step
 * acceleration, and uses separated aerodynamic drag and per-wheel rolling resistance.
 */
#ifndef DRIFTY_CONFIG_H
#define DRIFTY_CONFIG_H

/* -------------------------------------------------------------------------------------
 * Render scale (rendering only — never read by simulation code)
 * ------------------------------------------------------------------------------------- */

/* Dimensionless render scale: how many screen pixels one world meter occupies.
 * Overridable at build time (-DPIXELS_PER_METER=48.0f) purely to prove that changing it
 * alters nothing but visual size. It is the default for Game.renderPixelsPerMeter. */
#ifndef PIXELS_PER_METER
#define PIXELS_PER_METER 24.0f
#endif

/* -------------------------------------------------------------------------------------
 * Fixed timestep
 * ------------------------------------------------------------------------------------- */

#define FIXED_HZ 120               /* fixed updates per second */
#define FIXED_DT_S (1.0f / 120.0f) /* seconds per fixed update */
#define MAX_PHYSICS_STEPS 8        /* substep cap per render frame */
#define MAX_FRAME_TIME_S 0.25f     /* seconds; frame-time clamp */

/* -------------------------------------------------------------------------------------
 * Deterministic input recording
 * ------------------------------------------------------------------------------------- */

/* Fixed-capacity ring buffer, sized in fixed ticks. 14400 ticks = 120.0 s at 120 Hz.
 * Overflow behaviour is a documented ring: the oldest tick is discarded and counted in
 * ReplayBuffer.overwrittenTicks. See src/game/replay.h. */
#define REPLAY_CAPACITY_TICKS 14400
/* -------------------------------------------------------------------------------------
 * Phase 1/2 vehicle physics (SI units)
 * ------------------------------------------------------------------------------------- */

#define GRAVITY_MPS2 9.80665f
#define AIR_DENSITY_KGM3 1.225f /* kg/m^3, sea-level standard atmosphere */

#define VEH_MASS_KG 1200.0f
#define VEH_YAW_INERTIA_KGM2 1800.0f
#define VEH_CG_TO_FRONT_M 1.15f
#define VEH_CG_TO_REAR_M 1.40f
#define VEH_WHEELBASE_M (VEH_CG_TO_FRONT_M + VEH_CG_TO_REAR_M)
#define VEH_CG_HEIGHT_M 0.50f
#define VEH_TRACK_FRONT_M 1.48f
#define VEH_TRACK_REAR_M 1.46f

#define WHEEL_RADIUS_M 0.31f
#define WHEEL_INERTIA_KGM2 1.20f

/* Phase 2 geometry — primary inputs. Derived length = wheelbase + overhangs. */
#define VEH_FRONT_OVERHANG_M 0.78f
#define VEH_REAR_OVERHANG_M 0.82f
#define VEH_WIDTH_OVERALL_M 1.70f /* 2 * VEHICLE_BODY_HALF_WIDTH_M */
#define VEH_HEIGHT_OVERALL_M 1.45f
#define VEH_RIDE_HEIGHT_FRONT_M 0.140f
#define VEH_RIDE_HEIGHT_REAR_M 0.140f
/* Cowl / backlight X in the layout frame (axle midpoint origin, +X forward). */
#define VEH_COWL_X_M 0.55f
#define VEH_BACKLIGHT_X_M (-0.55f)
/* Open cargo bed measured forward from the tail; 0 means the vehicle has none.
 *
 * This exists because a bed and a boot are the same measurement when read off the backlight
 * station, and the appearance grammar previously had to guess between them from
 * (backlight - tail) / length alone. It guessed "bed" on 78 of the 100 corpus vehicles, 74 of
 * which are not trucks. A decklid IS the stretch of body behind the rear glass on every
 * three-box car, so no threshold on that ratio can separate the two. Only the vehicle itself
 * knows, so the vehicle now says. */
#define VEH_BED_LENGTH_M 0.0f
/* Phase B silhouette endpoints and arch flare (SI metres). Pixel effects are stated against
 * the 13.2 px/m world scale (PIXELS_PER_METER 24 x CAMERA_BASE_ZOOM 0.55): nose_width
 * 0.8..2.2 m = 11..29 px; tail_width 0.8..2.4 m = 11..32 px; shoulder_x moves the max-width
 * station over ~53 px of body length at stock; fender_flare_front 0..0.12 m = 0..1.6 px;
 * fender_flare_rear 0..0.15 m = 0..2 px.
 *
 * The three station defaults are chosen against the reference measurement rather than by eye.
 * Across the 77-sprite reference set a top-down car closes ~6 px of width over its last 15% of
 * length and gains ~10 px over its first 15%; the pre-Phase-B derived taper managed 2 px and
 * 8 px and left most of the corpus near front-to-back mirror symmetry. The candidate triple
 * was picked by sweeping nose width, tail width and the shoulder station over the recorded
 * corpus hull stations (artifacts/corpus-cards/cards.json, 13.2 px/m) and keeping the one that
 * cleared the taper and mirror-symmetry gates with the most tail closure. Nose is narrower
 * than tail because a bonnet is narrower than a boot, and the shoulder sits 0.9 m behind the
 * axle midpoint: the rear-door station where a real body is widest. The post-change fleet
 * measurements live in artifacts/visual/diagnostics.txt, not in this comment. */
#define VEH_NOSE_WIDTH_M 0.85f
#define VEH_TAIL_WIDTH_M 1.10f
#define VEH_SHOULDER_X_M (-0.90f) /* layout frame (axle midpoint origin, +X forward) */
#define VEH_FENDER_FLARE_FRONT_M 0.0f
#define VEH_FENDER_FLARE_REAR_M 0.0f

/* Phase C greenhouse primaries. Longitudinal stations use the layout frame; all other
 * geometry is SI. The roof defaults reproduce the pre-Phase-C stock roof exactly:
 * 0.890 m long x 0.924 m wide at 13.2 px/m. roof_width is the full physical width, so
 * tumblehome inset is (widthOverallM - roofWidthM) / 2. */
#define VEH_ROOF_START_X_M 0.8185f
#define VEH_ROOF_END_X_M (-0.0715f)
#define VEH_ROOF_WIDTH_M 0.9240f
#define VEH_WINDSCREEN_RAKE_RAD 0.7853981634f     /* 45 deg from vertical */
#define VEH_WINDSCREEN_RAKE_MIN_RAD 0.3490658504f /* 20 deg */
#define VEH_WINDSCREEN_RAKE_MAX_RAD 1.2217304764f /* 70 deg */
#define VEH_BACKLIGHT_RAKE_RAD 0.7853981634f      /* 45 deg from vertical */
#define VEH_BACKLIGHT_RAKE_MIN_RAD 0.2617993878f  /* 15 deg */
#define VEH_BACKLIGHT_RAKE_MAX_RAD 1.3089969390f  /* 75 deg */
#define VEH_SIDE_WINDOW_COUNT 2.0f
#define VEH_QUARTER_WINDOW_LENGTH_M 0.0f
#define VEH_SUNROOF_LENGTH_M 0.0f
#define VEH_DOOR_COUNT 4.0f
#define VEH_CABIN_ROWS 2.0f
#define VEH_ROOF_TYPE 0.0f /* 0=fixed, 1=targa, 2=convertible */
/* Derived readout default: the staged refresh computes exactly this sum. */
#define VEH_LENGTH_OVERALL_M (VEH_WHEELBASE_M + VEH_FRONT_OVERHANG_M + VEH_REAR_OVERHANG_M)
/* Frontal area fill of width*height that recovers FRONTAL_AREA_M2 at stock. */
#define VEH_FRONTAL_AREA_FILL 0.7707910751f

/* Mass particles in the layout frame. Tuned so staged refresh recovers the stock mass,
 * CG distances, CG height, and yaw inertia (see VEH_YAW_RADIUS_FACTOR). */
#define MASS_ENGINE_KG 160.0f
#define MASS_ENGINE_X_M 1.10f
#define MASS_ENGINE_Z_M 0.40f
#define MASS_GEARBOX_KG 70.0f
#define MASS_GEARBOX_X_M 0.50f
#define MASS_GEARBOX_Z_M 0.35f
#define MASS_FUEL_KG 60.0f
#define MASS_FUEL_X_M (-1.10f)
#define MASS_FUEL_Z_M 0.30f
#define MASS_DRIVER_KG 90.0f
#define MASS_DRIVER_X_M 0.20f
#define MASS_DRIVER_Z_M 0.55f
#define MASS_CHASSIS_KG 820.0f
#define MASS_CHASSIS_X_M (-0.0158536585f)
#define MASS_CHASSIS_Z_M 0.5414634146f
/* Izz = sum m*(x-x_cg)^2 + M*(factor*wheelbase)^2. Factor recovers VEH_YAW_INERTIA_KGM2. */
#define VEH_YAW_RADIUS_FACTOR 0.4429874092f

/* Tire designation (ISO metric). Loaded radius = unloaded * TIRE_LOAD_RADIUS_FACTOR. */
#define TIRE_SECTION_WIDTH_MM 225.0f
#define TIRE_ASPECT_RATIO_PCT 45.0f
#define TIRE_RIM_DIAMETER_IN 17.0f
#define TIRE_RIM_WIDTH_IN 8.0f
#define TIRE_PRESSURE_KPA 220.0f
#define TIRE_LOAD_RADIUS_FACTOR 0.9774554627f

/* Suspension / stance (visual + roll-share inputs). */
#define SUSP_CAMBER_FRONT_RAD (-0.0175f)
#define SUSP_CAMBER_REAR_RAD (-0.0175f)
#define SUSP_TOE_FRONT_RAD 0.0035f
#define SUSP_TOE_REAR_RAD 0.0017f
#define SUSP_CASTER_FRONT_RAD 0.087f
#define SUSP_CASTER_REAR_RAD 0.052f
#define SUSP_WHEEL_RATE_FRONT_NPM 28000.0f
#define SUSP_WHEEL_RATE_REAR_NPM 26000.0f
#define SUSP_ANTI_ROLL_FRONT_NPM 18000.0f
#define SUSP_ANTI_ROLL_REAR_NPM 16000.0f
#define SUSP_TRAVEL_FRONT_M 0.090f
#define SUSP_TRAVEL_REAR_M 0.095f
#define SUSP_ROLL_CENTRE_FRONT_M 0.080f
#define SUSP_ROLL_CENTRE_REAR_M 0.100f

/* Wheel offset (ET, mm) and brake hardware. */
#define WHEEL_OFFSET_ET_FRONT_MM 35.0f
#define WHEEL_OFFSET_ET_REAR_MM 35.0f
#define BRAKE_DISC_RADIUS_FRONT_M 0.150f
#define BRAKE_DISC_RADIUS_REAR_M 0.140f
#define BRAKE_PAD_FRICTION 0.38f
#define BRAKE_PAD_AREA_M2 0.0045f
#define BRAKE_HYDRAULIC_PRESSURE_PA 1.0e7f
/* Scale so padFriction*area*pressure*(rF+rR)*scale == MAX_BRAKE_TORQUE_NM at stock. */
#define BRAKE_TORQUE_SCALE 0.5730659029f

/* Aero appendage inputs (visual + future downforce). */
#define AERO_LIFT_COEF_FRONT 0.05f
#define AERO_LIFT_COEF_REAR 0.10f
#define AERO_REF_AREA_FRONT_M2 0.40f
#define AERO_REF_AREA_REAR_M2 0.55f
#define AERO_COP_X_M (-0.20f)

/* Drivetrain layout: 0=RWD, 1=FWD, 2=AWD. */
#define DRIVETRAIN_LAYOUT_DEFAULT 0.0f
#define DRIVETRAIN_FRONT_TORQUE_SPLIT 0.0f
#define ENGINE_CYLINDERS 4.0f
#define ENGINE_DISPLACEMENT_L 2.0f

#define STEER_MAX_RAD 0.70f
#define STEER_RATE_RAD_S 5.0f
#define STEER_RETURN_RATE_RAD_S 7.0f
#define STEER_SPEED_REF_MPS 8.0f /* m/s; below this, full-rate steering */
#define STEER_SPEED_MIN_FACTOR \
    1.0f /* dimensionless; minimum steering rate at high speed (1.0 = disabled) */

#define DRAG_COEFFICIENT 0.32f
#define ROLLING_RESISTANCE_COEF 0.015f

/* First-order corner frequency of the load-transfer acceleration filter, in hertz. It
 * models the suspension's finite pitch response: a step in longitudinal force does not
 * move the axle loads instantaneously. Registered as a tunable. */
#define LOAD_FILTER_RATE_HZ 20.0f

/* Normalized nonlinear tire curves. */
#define TIRE_B_LAT_FRONT 10.0f
#define TIRE_C_LAT_FRONT 1.45f
#define TIRE_MU_LAT_FRONT 1.30f
#define TIRE_B_LAT_REAR 10.0f
#define TIRE_C_LAT_REAR 1.45f
#define TIRE_MU_LAT_REAR 1.20f
#define TIRE_B_LONG 12.0f
#define TIRE_C_LONG 1.55f
#define TIRE_MU_LONG_SCALE 1.00f

#define MAX_GEARS 8
#define ENGINE_CURVE_POINTS 7

#define GEAR_RATIOS { 3.55f, 2.05f, 1.38f, 1.00f, 0.82f }
#define GEAR_COUNT 5
#define REVERSE_GEAR_RATIO 3.20f
#define FINAL_DRIVE_RATIO 4.10f
#define DRIVETRAIN_EFFICIENCY 0.90f
#define ENGINE_IDLE_RPM 900.0f
#define ENGINE_REDLINE_RPM 7000.0f
#define ENGINE_BRAKING_TORQUE_NM 35.0f
#define ENGINE_TORQUE_CURVE_NM { 140.0f, 200.0f, 240.0f, 255.0f, 250.0f, 230.0f, 195.0f }

#define MAX_BRAKE_TORQUE_NM 3000.0f
#define BRAKE_BIAS_FRONT 0.62f
#define HANDBRAKE_TORQUE_NM 1800.0f

/* Rate at which the kinematic low-speed model pulls lateral velocity and yaw rate onto
 * their geometric targets, in hertz. Not a resistance term and not a handling tunable: it
 * is the stiffness of the model blend that keeps the vehicle stable below LOW_SPEED_BEGIN,
 * and changing it interactively would change what "kinematic" means mid-run. */
#define LOW_SPEED_RESPONSE_RATE_HZ 10.0f

#define LOW_SPEED_EPSILON_MPS 0.50f
#define SLIP_SPEED_EPSILON_MPS 1.00f
#define SLIP_RATIO_CLAMP 4.00f
#define LOW_SPEED_BEGIN_MPS 1.50f
#define LOW_SPEED_END_MPS 3.00f
#define MIN_NORMAL_LOAD_N 50.0f
#define FRICTION_TOLERANCE 0.001f
#define MAX_SAFE_SPEED_MPS 120.0f
#define MAX_SAFE_YAW_RATE_RADS 20.0f

/* Phase 3 numerical guards. These bound the resistance model rather than shaping it, so
 * they stay compile-time constants: a slider that can make drag point along +v, or let a
 * resistance force reverse the car inside one tick, does not describe a different car —
 * it describes a broken integrator.
 *
 *   RESISTANCE_EPSILON_MPS       direction denominator floor; below it there is no
 *                                well-defined direction to oppose, so the force is zero.
 *   MAX_LOAD_TRANSFER_FRACTION   safety tripwire, as a multiple of mass * g. Physical
 *                                transfer peaks near 0.3 of that; 1.5 means something is
 *                                badly wrong, not merely aggressive.
 *   RESISTANCE_POWER_TOLERANCE_W slack for the "resistance never adds energy" assertion,
 *                                in watts, absorbing float rounding in the dot product.
 *                                Aero is checked against body velocity.
 *   RESISTANCE_FORCE_TOLERANCE_N slack when comparing stored per-wheel rolling-resistance
 *                                magnitudes to a fresh pure-helper recompute (newtons).
 */
#define RESISTANCE_EPSILON_MPS 1.0e-4f
#define MAX_LOAD_TRANSFER_FRACTION 1.50f
#define RESISTANCE_POWER_TOLERANCE_W 1.0e-2f
#define RESISTANCE_FORCE_TOLERANCE_N 1.0e-2f

/* Phase 5 collision --------------------------------------------------------- */

/* Capsule covering the car footprint: two circles at front/rear axle plus the connecting
 * body. The radius of each circle is the half-width of the car body, smaller than the track
 * width which is wheel-to-wheel center distance. */
#define VEHICLE_BODY_HALF_WIDTH_M 0.85f /* m; collision capsule circle radius */
#define COLLISION_RESTITUTION 0.30f     /* dimensionless; barrier bounce (< 1) */
#define COLLISION_FRICTION 0.50f        /* dimensionless; Coulomb friction at impact */

/* Scoring lockout after a barrier impact (Phase 6). Timer starts counting down from this. */
#define CRASH_LOCKOUT_S 1.0f /* seconds; post-impact scoring suspension */

/* Drift classification thresholds (Phase 6). Gameplay rules, not physical tunables:
 * these are compile-time constants that never feed back into the force model. */
#define MIN_DRIFT_SPEED_MPS 5.0f      /* m/s; below this speed a slide is not scored */
#define MIN_DRIFT_ANGLE_RAD 0.175f    /* rad (~10 deg); minimum body sideslip for scoring */
#define MIN_REAR_SLIP_RAD 0.12f       /* rad; minimum rear-axle slip for scoring */
#define MIN_DRIFT_YAW_RATE_RADS 0.25f /* rad/s; minimum yaw rate for scoring */
#define SPIN_CUTOFF_RAD 1.48f         /* rad (~85 deg); beyond this the car has spun */

/* Score accumulation (Phase 6). Points are accrued per fixed tick while scoringDrift is true. */
#define SCORE_BASE_RATE 100.0f    /* points/second at full factors and base multiplier */
#define SCORE_SPEED_REF_MPS 35.0f /* m/s; the speed at which speedFactor saturates */
#define COMBO_GRACE_S 1.50f /* s; reset multiplier after this long outside a scoring drift */
#define MAX_VALID_SCORE 100000000L /* upper bound for file-load validation and clamping */

/* Phase 6 results trigger (presentation only; no effect on simulation). */
#define RESULTS_TARGET_LAPS 3 /* laps to complete before entering STATE_RESULTS */

/* Phase 4 four-wheel fidelity ---------------------------------------------- */
#define TIRE_LOAD_SENSITIVITY_K 0.02f /* dimensionless; realistic ~0.02. 0 disables */
/* N; the staged refresh derives this as m*g/4, so the constant must be that expression
 * rather than a rounded literal — the params scenario compares the two exactly. */
#define TIRE_LOAD_REF_PER_WHEEL_N (VEH_MASS_KG * GRAVITY_MPS2 * 0.25f)
#define TIRE_RELAXATION_LENGTH_M 0.30f   /* m; realistic 0.20..0.50. 0 disables */
#define ACKERMANN_PERCENT 1.00f          /* dimensionless; 0=parallel, 1=true Ackermann */
#define DIFFERENTIAL_MODE_DEFAULT 2      /* 0=LOCKED, 1=OPEN, 2=LSD */
#define DIFFERENTIAL_BIAS_RATIO 2.0f     /* dimensionless; LSD slower/faster cap */
#define DIFFERENTIAL_PRELOAD_NM 60.0f    /* N*m; LSD clutch preload */
#define DIFF_OMEGA_EPSILON_RAD_S 1.0e-3f /* rad/s; LSD omega-difference deadband */
#define ROLL_STIFFNESS_FRONT_FRACTION \
    0.50f                              /* dimensionless 0..1; front axle roll-moment share */
#define SURFACE_REFERENCE_MU_LAT 1.30f /* asphalt lateral mu; tireMuLat* are absolute vs this */
#define SURFACE_REFERENCE_MU_LONG \
    1.35f /* asphalt longitudinal mu; documentation reference only */
#define RELAXATION_VX_FLOOR_MPS 0.50f /* m/s; floor for relaxation vx denominator */

/* -------------------------------------------------------------------------------------
 * Window and presentation (no effect on the simulation layer)
 * ------------------------------------------------------------------------------------- */

#define SCREEN_W 1280 /* pixels */
#define SCREEN_H 720  /* pixels */
#define TARGET_FPS 60 /* render frames per second */

#define RELOAD_FLASH_S 2.0f /* seconds the "module reloaded" HUD notice persists */

/* Phase 6 particles --------------------------------------------------------- */

#define MAX_PARTICLES 512     /* total slots in the round-robin particle pool */
#define PARTICLE_LIFE_S 0.80f /* seconds a smoke particle lives before fading out */

/* Phase 6 camera drift zoom ------------------------------------------------- */

#define CAMERA_BASE_ZOOM 0.55f      /* dimensionless; >1 magnifies (zooms in) */
#define CAMERA_ZOOM_RANGE 0.10f     /* dimensionless; subtracted at full drift -> 0.45 */
#define CAMERA_MIN_ZOOM 0.30f       /* dimensionless; must stay > 0 */
#define CAMERA_ZOOM_RATE 4.0f       /* 1/second smoothing rate */
#define CAMERA_LOOKAHEAD 0.25f      /* seconds of velocity lookahead (reserved) */
#define DRIFT_ZOOM_REF_RAD 0.70f    /* sideslip mapped to full zoom-out */
#define SLIDE_USAGE_THRESHOLD 0.98f /* dimensionless friction usage = physically sliding */

/* Pixel-art world pass ------------------------------------------------------
 *
 * THE SCALE CHAIN, END TO END. Five numbers have to agree for the world to be pixel art
 * rather than a smoothly-filtered approximation of it, and this is where they are reconciled:
 *
 *   PIXELS_PER_METER       24      the render layer's world scale, held in
 *                                  Game.renderPixelsPerMeter (src/core/units.h consumes it)
 *   CAMERA_BASE_ZOOM       0.55    the Camera2D zoom at rest
 *   world scale            13.2    PIXELS_PER_METER * CAMERA_BASE_ZOOM px/m — the scale the
 *                                  world is rasterized at inside the low-resolution target
 *   sprite bake scale      13.2    the SAME number, so a vehicle texel is exactly one target
 *                                  pixel at rest and the sprite is never resampled
 *   PIXEL_ART_UPSCALE      2       integer, nearest-neighbour, applied once to the finished
 *                                  target: 640x360 -> 1280x720
 *
 * WHY 13.2 AND NOT SOMETHING ROUNDER. It is what the vehicle grammar is calibrated to. The
 * corpus distinctness thresholds, the one-pixel visibility floor behind every presentation
 * gain, and the headless contact sheet are all stated at PIXELS_PER_METER * CAMERA_BASE_ZOOM.
 * Rendering the world at any other scale would mean the player looks at a different car from
 * the one the tests gate.
 *
 * WHY 2 AND NOT 3. 1280 and 720 must both divide by it exactly, or the final blit is
 * fractional and the pixel grid crawls under camera motion — which is the whole thing this
 * pass exists to prevent. 2 and 4 divide 1280x720; 3 does not.
 *
 * WHAT THIS CHANGES FOR THE PLAYER. The world is drawn into a 640x360 target and doubled, so
 * the on-screen scale becomes 26.4 px/m and the visible world halves. CAMERA_BASE_ZOOM itself
 * is deliberately NOT touched: the alternative — halving it to hold the old framing — would
 * put the world back at 6.6 px/m, where a car is 27 px long and most of the grammar is below
 * the visibility floor.
 *
 * WHAT IS NOT UPSCALED. The HUD, raygui and the Physics Lab draw at native resolution on top
 * of the blit. Text through a 2x nearest-neighbour upscale would be unreadable.
 */
#define PIXEL_ART_UPSCALE 2 /* integer; must divide SCREEN_W and SCREEN_H exactly */
#define PIXEL_ART_TARGET_W (SCREEN_W / PIXEL_ART_UPSCALE)
#define PIXEL_ART_TARGET_H (SCREEN_H / PIXEL_ART_UPSCALE)

#endif /* DRIFTY_CONFIG_H */
