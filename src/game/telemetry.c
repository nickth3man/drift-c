#include "game/telemetry.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(_WIN32)
#include <direct.h>
#define drifty_mkdir(path) _mkdir(path)
#else
#define drifty_mkdir(path) mkdir((path), 0775)
#endif

#define TELEMETRY_HEADER                                                                      \
    "tick,time_s,position_x_m,position_y_m,heading_rad,"                                      \
    "velocity_longitudinal_mps,velocity_lateral_mps,speed_mps,yaw_rate_rad_s,"                \
    "steering_angle_rad,engine_rpm,selected_gear,front_slip_angle_rad,rear_slip_angle_rad,"   \
    "front_slip_ratio,rear_slip_ratio,front_wheel_omega_rad_s,rear_wheel_omega_rad_s,"        \
    "front_normal_load_n,rear_normal_load_n,front_fx_pure_n,rear_fx_pure_n,"                  \
    "front_fy_pure_n,rear_fy_pure_n,front_fx_limited_n,rear_fx_limited_n,"                    \
    "front_fy_limited_n,rear_fy_limited_n,front_friction_usage,rear_friction_usage,"          \
    "front_locked,rear_locked,drive_torque_nm,front_brake_torque_nm,"                         \
    "rear_brake_torque_nm,handbrake_torque_nm,total_force_x_n,total_force_y_n,yaw_torque_nm," \
    "body_sideslip_rad,low_speed_blend,substep_count,backlog_drops,state_checksum,"           \
    "static_front_load_n,static_rear_load_n,dynamic_front_load_n,dynamic_rear_load_n,"        \
    "load_transfer_n,previous_long_accel_mps2,filtered_long_accel_mps2,"                      \
    "solved_long_accel_mps2,lateral_accel_mps2,aero_drag_n,aero_drag_x_n,aero_drag_y_n,"      \
    "rolling_resistance_n,rolling_resistance_x_n,rolling_resistance_y_n,"                     \
    "steering_input,throttle_input,brake_input,handbrake_input,"                              \
    "surface_front_left,surface_front_right,surface_rear_left,surface_rear_right"

const char *telemetry_header(void)
{
    return TELEMETRY_HEADER;
}

bool telemetry_ensure_dir(const char *dirPath)
{
    if (dirPath == NULL || dirPath[0] == '\0') return false;

    /* Creates intermediate components, because the output roots are nested now
     * ("artifacts/telemetry") and a clean checkout has no "artifacts" at all. A single mkdir
     * would fail with ENOENT there and every writer would report a permission-looking error
     * for what is really a missing parent. */
    char buffer[512];
    const size_t length = strlen(dirPath);
    if (length >= sizeof(buffer)) {
        fprintf(stderr, "TELEMETRY: directory path too long: '%s'\n", dirPath);
        return false;
    }
    memcpy(buffer, dirPath, length + 1);

    for (char *c = buffer; *c != '\0'; c++) {
        if (*c != '/' && *c != '\\') continue;
        if (c == buffer) continue; /* leading separator: not a component */

        const char separator = *c;
        *c = '\0';
        /* A bare drive prefix such as "C:" is not a directory anyone can create. */
        const bool isDrivePrefix = (c - buffer == 2 && buffer[1] == ':');
        if (!isDrivePrefix && drifty_mkdir(buffer) != 0 && errno != EEXIST) {
            fprintf(stderr, "TELEMETRY: could not create directory '%s': %s\n", buffer,
                    strerror(errno));
            *c = separator;
            return false;
        }
        *c = separator;
    }

    if (drifty_mkdir(buffer) == 0) return true;
    if (errno == EEXIST) return true;

    fprintf(stderr, "TELEMETRY: could not create directory '%s': %s\n", dirPath,
            strerror(errno));
    return false;
}

bool telemetry_open(TelemetryWriter *writer, const char *path)
{
    if (writer == NULL) return false;

    writer->file = NULL;
    writer->rowCount = 0;
    writer->failed = false;

    if (path == NULL || path[0] == '\0') {
        writer->failed = true;
        return false;
    }

    writer->file = fopen(path, "wb");
    if (writer->file == NULL) {
        fprintf(stderr, "TELEMETRY: could not open '%s' for writing: %s\n", path,
                strerror(errno));
        writer->failed = true;
        return false;
    }

    if (fprintf(writer->file, "%s\n", TELEMETRY_HEADER) < 0) {
        fprintf(stderr, "TELEMETRY: could not write header to '%s'\n", path);
        writer->failed = true;
        fclose(writer->file);
        writer->file = NULL;
        return false;
    }

    return true;
}

bool telemetry_write_row(TelemetryWriter *writer, const TelemetryRow *row)
{
    if (writer == NULL || writer->file == NULL || writer->failed || row == NULL) return false;

    const int written = fprintf(
        writer->file,
        "%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%" PRIu32 ","
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%"
        ".6f,%.6f,%d,%d,%d,%d\n",
        row->tick, row->timeS, (double)row->positionXM, (double)row->positionYM,
        (double)row->headingRad, (double)row->velocityLongitudinalMps,
        (double)row->velocityLateralMps, (double)row->speedMps, (double)row->yawRateRadS,
        (double)row->steeringAngleRad, (double)row->engineRpm, row->selectedGear,
        (double)row->frontSlipAngleRad, (double)row->rearSlipAngleRad,
        (double)row->frontSlipRatio, (double)row->rearSlipRatio,
        (double)row->frontWheelOmegaRadS, (double)row->rearWheelOmegaRadS,
        (double)row->frontNormalLoadN, (double)row->rearNormalLoadN, (double)row->frontFxPureN,
        (double)row->rearFxPureN, (double)row->frontFyPureN, (double)row->rearFyPureN,
        (double)row->frontFxLimitedN, (double)row->rearFxLimitedN, (double)row->frontFyLimitedN,
        (double)row->rearFyLimitedN, (double)row->frontFrictionUsage,
        (double)row->rearFrictionUsage, row->frontLocked, row->rearLocked,
        (double)row->driveTorqueNm, (double)row->frontBrakeTorqueNm,
        (double)row->rearBrakeTorqueNm, (double)row->handbrakeTorqueNm,
        (double)row->totalForceXN, (double)row->totalForceYN, (double)row->yawTorqueNm,
        (double)row->bodySideslipRad, (double)row->lowSpeedBlend, row->substepCount,
        row->backlogDrops, row->stateChecksum, (double)row->staticFrontLoadN,
        (double)row->staticRearLoadN, (double)row->dynamicFrontLoadN,
        (double)row->dynamicRearLoadN, (double)row->loadTransferN,
        (double)row->previousLongAccelMps2, (double)row->filteredLongAccelMps2,
        (double)row->solvedLongAccelMps2, (double)row->lateralAccelMps2, (double)row->aeroDragN,
        (double)row->aeroDragXN, (double)row->aeroDragYN, (double)row->rollingResistanceN,
        (double)row->rollingResistanceXN, (double)row->rollingResistanceYN,
        (double)row->steeringInput, (double)row->throttleInput, (double)row->brakeInput,
        (double)row->handbrakeInput, row->surfaceFrontLeft, row->surfaceFrontRight,
        row->surfaceRearLeft, row->surfaceRearRight);
    if (written < 0) {
        fprintf(stderr, "TELEMETRY: write failed after %ld rows\n", writer->rowCount);
        writer->failed = true;
        return false;
    }

    writer->rowCount++;
    return true;
}

bool telemetry_close(TelemetryWriter *writer)
{
    if (writer == NULL) return false;
    if (writer->file == NULL) return !writer->failed;

    const bool closedCleanly = (fclose(writer->file) == 0);
    writer->file = NULL;

    if (!closedCleanly) {
        fprintf(stderr, "TELEMETRY: fclose failed: %s\n", strerror(errno));
        writer->failed = true;
    }

    return !writer->failed;
}
