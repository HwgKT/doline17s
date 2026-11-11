#include "pid_controller.h"
#include "map1_route_lock.h"
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>
#include <math.h>

// Sensor 0..7 are left of center; 8..15 are right of center.
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f,
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f
};

// Optimised tune for sharp-turn responsiveness.  Gains are still live-tuneable
// via the SPP KP:/KI:/KD: commands.
static PID_Config_t pidConfig = {1.6f, 0.0f, 0.8f, 100, 255, -100};
static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0, integralError = 0, lastCorrection = 0;
static float lastPTerm = 0, lastITerm = 0, lastDTerm = 0;
static float lastKnownPosition = 0;
static uint32_t lastComputeTimeUs = 0;
static bool derivativeHistoryValid = false;
static float lastDashedCorrection = 0;   // remembered PID output for dashed-line coast

namespace {
constexpr uint32_t LINE_ACQUIRE_MS = 10;
constexpr uint32_t LINE_LOSS_MS = 30;           // was 40 — faster loss detection reduces stutter duration
// A line that disappeared at the outer edge during an active sharp turn is
// not a centered dashed gap. Pivot immediately rather than driving both
// wheels forward for the usual 30 ms loss-confirmation window.
constexpr float EDGE_CORNER_IMMEDIATE_SEARCH_ERROR = 55.0f;
constexpr uint32_t LINE_SEARCH_TIMEOUT_MS = 2000;
constexpr uint32_t STARTUP_LOSS_GRACE_MS = 5000;
// Finish requires ALL 16 sensors to see black on 3 consecutive PID samples.
constexpr uint8_t FULL_LINE_MIN_SENSORS = N_SENSORS;
constexpr uint8_t FINISH_CONFIRM_FRAMES = 3;
// START can be on WHITE, a narrow line, or an all-black pad. Never treat
// any 16/16-black detection as FINISH until a narrow, centered track has
// been confirmed continuously after START. No encoder distance is assumed.
constexpr uint8_t START_PAD_EXIT_MAX_BLACK = 10;
constexpr uint16_t START_PAD_CENTER_MASK = 0x03C0U; // central eyes S6..S9
constexpr float START_PAD_CENTER_TOLERANCE = 22.0f;
constexpr uint32_t START_PAD_EXIT_CONFIRM_MS = 30;
constexpr int16_t DROPOUT_FORWARD_PWM = 95;    // was 65 — higher coast speed maintains momentum over dashed lines
constexpr int16_t LINE_SEARCH_PWM = 90;
// Map 1 early corner: fixed SEARCH-style pivot, stronger reverse inner wheel.
// No change to ordinary SEARCH or line PID. Active from AUTO start
// (once the START pad has been exited) until 12000 progress ticks.
constexpr int32_t EARLY_TURN_MIN_TICKS = 0;
constexpr int32_t EARLY_TURN_MAX_TICKS = 12000; // exclusive: no new early turns at >=12000
constexpr int16_t EARLY_TURN_INNER_REVERSE_PWM = 100;
constexpr int16_t EARLY_TURN_OUTER_FORWARD_PWM = 90;
constexpr uint32_t EARLY_TURN_TREND_WINDOW_MS = 150;
constexpr uint8_t EARLY_TURN_EDGE_CONFIRM_FRAMES = 3;
// Fallback for fast center-to-isolated-edge transitions (two valid samples).
constexpr uint8_t EARLY_TURN_DIRECT_EDGE_FRAMES = 2;
// MAP 1 experiment: faster pivot only at the long left corner near 1,311 ticks.
constexpr int16_t MAP1_LEFT_SEARCH_TEST_PWM = 90;
constexpr int32_t MAP1_LEFT_SEARCH_TEST_START_ENC = 1200;
constexpr int32_t MAP1_LEFT_SEARCH_TEST_END_ENC = 1700;
constexpr uint8_t SEARCH_REACQUIRE_FRAMES = 3;
constexpr uint8_t REACQUIRE_DERIVATIVE_HOLD_FRAMES = 4;
constexpr uint8_t REACQUIRE_SPEED_RAMP_FRAMES = 4;
// Named-track dashed-line coast. Encoder positions are relative to AUTO start.
// These are provisional windows extracted from the 2026-10-31 map logs.
// TEMPORARY PID-only test mode for mainHighSpeedv2 (default OFF).
// When disabled, neither the encoder-triggered high-speed PWM envelope nor
// the encoder-position GAP windows can change the motor commands.
// Encoder readings remain available for logging/diagnostics.
// Set -DDOLINE_ENABLE_ENCODER_SPEED_PROFILE=1 to restore original behavior.
#ifndef DOLINE_ENABLE_ENCODER_SPEED_PROFILE
#define DOLINE_ENABLE_ENCODER_SPEED_PROFILE 0
#endif
// The old encoder-position speed envelope is disabled. Encoder is now used
// for wheel-speed feedback and (separately) the sensor mask stages only.
constexpr bool ENCODER_SPEED_PROFILE_ENABLED = false;
constexpr uint32_t GAP_TIMEOUT_MS = 100;
// First straight trapezoid in relative motor-encoder ticks.
// An experimental open-loop PWM envelope, not closed-loop wheel velocity.
constexpr int32_t HS_RAMP_END_ENC = 300;
// Shared first-straight corner profile for MAP 1/2/3.
constexpr int32_t HS_BRAKE_START_ENC = 1800;        // Peak 100..229
constexpr int32_t HS_EARLY_BRAKE_START_ENC = 1600;  // Peak 230..239
constexpr int16_t HS_EARLY_BRAKE_PEAK_THRESHOLD = 230;
constexpr int32_t HS_VERY_EARLY_BRAKE_START_ENC = 1400; // Peak 240..255
constexpr int16_t HS_VERY_EARLY_BRAKE_PEAK_THRESHOLD = 240;
constexpr int32_t HS_BRAKE_END_ENC = 2300;
constexpr int32_t HS_HOLD_MIN_END_ENC = 3100;
constexpr float HS_CORNER_HOLD_PWM = 110.0f;
constexpr float HS_HOLD_CENTER_ERROR = 15.0f; // valid line position near center
int16_t highSpeedPeakPwm = 240; // All three MAPs default to peak PWM 240.
enum HighSpeedStage : uint8_t {
    HS_DISABLED=0, HS_ACCEL=1, HS_CRUISE=2, HS_BRAKE=3,
    HS_FINISHED=4, HS_ABORTED=5, HS_CORNER_HOLD=6
};
HighSpeedStage highSpeedStage = HS_DISABLED;
bool highSpeedAborted = false;
int32_t highSpeedMaxProgress = 0;

static int32_t highSpeedBrakeStart() {
    // Peak can only change when AUTO is disabled: the window remains fixed
    // throughout a run for all three map profiles.
    if (highSpeedPeakPwm >= HS_VERY_EARLY_BRAKE_PEAK_THRESHOLD)
        return HS_VERY_EARLY_BRAKE_START_ENC;
    return highSpeedPeakPwm >= HS_EARLY_BRAKE_PEAK_THRESHOLD
        ? HS_EARLY_BRAKE_START_ENC : HS_BRAKE_START_ENC;
}

static float highSpeedForwardTarget(int32_t ticks, float basePwm) {
    const float peak = fmaxf(basePwm, (float)highSpeedPeakPwm);
    // HOLD target is independent of BASE (e.g. BASE=100, HOLD=110).
    // Never turn braking into acceleration if HS_PEAK is below 110.
    const float holdPwm = fminf(HS_CORNER_HOLD_PWM, peak);
    if (ticks <= 0) return basePwm;
    if (ticks < HS_RAMP_END_ENC) {
        return basePwm + (peak - basePwm) *
            ((float)ticks / (float)HS_RAMP_END_ENC);
    }
    const int32_t brakeStart = highSpeedBrakeStart();
    if (ticks < brakeStart) return peak;
    if (ticks < HS_BRAKE_END_ENC) {
        return peak +
            (holdPwm - peak) *
            ((float)(ticks - brakeStart) /
             (float)(HS_BRAKE_END_ENC - brakeStart));
    }
    // Keep low forward thrust until encoder progress and a valid centered
    // line sample satisfy the HOLD release conditions in pid_detect_case().
    // HS_FINISHED bypasses this envelope and returns to normal BASE.
    return holdPwm;
}
struct GapWindow { int32_t begin; int32_t end; };
constexpr GapWindow GAP_WINDOWS[3] = {
    {1500, 2100}, // MAP 1
    {1050, 1750}, // MAP 2
    {1000, 1700}  // MAP 3
};
uint8_t selectedMap = 0;  // setup() selects MAP 1; MAP:2/3 may override.
constexpr int16_t MAP_DEFAULT_BASE_PWM[3] = {130, 160, 150};
constexpr int16_t MAP_DEFAULT_HS_PEAK_PWM = 240;
// Fixed per-map BASE: no encoder-position scheduler, no late BOOST, and
// no per-section PWM caps. The independent wheel PI remains active.
// Map 1 only: arm the late BASE boost after 10,500 forward-progress ticks.
constexpr int32_t MAP1_LATE_BOOST_START_TICKS = 10500;
constexpr float MAP1_LATE_BOOST_CENTER_ERROR = 15.0f;
int32_t map1MaxProgress = 0;
bool map1LateBoostEngaged = false;
// Map 01 route lock assumes a repeatable START pose; enabled by default.
bool map1RouteLockEnabled = false; // Temporarily disabled by default for PID/sensor-mask testing; ROUTE_LOCK:1 to enable.
map1route::Tracker map1Route;
int32_t runEncoderOriginL = 0, runEncoderOriginR = 0;
bool gapActive = false;
uint32_t gapStartMs = 0;
int16_t gapHoldPwm = 0, lastNormalForwardPwm = 0;

static int32_t runEncoderProgress() {
    int32_t left = 0, right = 0;
    encoder_get_counts(&left, &right);
    const int64_t sum = ((int64_t)left - runEncoderOriginL) +
                        ((int64_t)right - runEncoderOriginR);
    return (int32_t)(sum / 2);
}

// PID-only sensor selection for Map 1. Keep physical ADC/line bits unmasked
// for START, FINISH, SEARCH, corner detection, and Route Lock observations.
// Encoder origins are captured at AUTO start (pid_set_auto_mode).
// User-configurable through SENSOR_PROFILE SPP commands while stopped.
// Values persist across AUTO runs but return to defaults after reboot.
int32_t map1SensorStage2Ticks = 650;
int32_t map1SensorStage3Ticks = 1350;
// Stage 1: disable S0..S4; stage 2: disable S9..S15; stage 3: all sensors.
uint16_t map1SensorStageMasks[3] = {0xFFE0U, 0x01FFU, 0xFFFFU};
constexpr uint16_t MAP1_SENSOR_MASK_STAGE3 = 0xFFFFU;
int32_t map1SensorMaxTicks = 0;
int32_t map2MaxProgress = 0; // Retained for Map 2 early-turn recognition.
// Map 2: four independent PID eye groups and three monotonic tick thresholds.
// Defaults all 16 eyes in every stage; does not affect physical observations.
int32_t map2SensorTickEnds[3] = {1000, 2000, 3000};
uint16_t map2SensorMasks[4] = {0xFFFFU,0xFFFFU,0xFFFFU,0xFFFFU};
int32_t map2SensorMaxTravel = 0;
// MAP 2 switches PID sensor masks at tick thresholds without stopping.
uint8_t map2SensorActiveStage = 0;
uint8_t map2HoldState = 0; // Legacy telemetry: always RUN (0).


static int32_t map1SensorTravelTicks() {
    int32_t left = 0, right = 0;
    encoder_get_counts(&left, &right);
    int64_t dl = (int64_t)left - runEncoderOriginL;
    int64_t dr = (int64_t)right - runEncoderOriginR;
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    const int64_t average = (dl + dr) / 2;
    return average > INT32_MAX ? INT32_MAX : (int32_t)average;
}

static uint16_t map1PidSensorMask();
static uint16_t map2PidSensorMask() {
    if (selectedMap != 2 || !autoMode) return 0xFFFFU;
    const int32_t traveled = map1SensorTravelTicks();
    if (traveled > map2SensorMaxTravel) map2SensorMaxTravel = traveled;
    while (map2SensorActiveStage < 3 &&
           map2SensorMaxTravel >= map2SensorTickEnds[map2SensorActiveStage]) {
        ++map2SensorActiveStage;
    }
    return map2SensorMasks[map2SensorActiveStage];
}
static uint16_t activePidSensorMask() {
    if (selectedMap == 2) return map2PidSensorMask();
    return map1PidSensorMask();
}
static uint16_t map1PidSensorMask() {
    if (selectedMap != 1 || !autoMode) return MAP1_SENSOR_MASK_STAGE3;
    const int32_t traveled = map1SensorTravelTicks();
    // A pivot or a temporary reversal must not reactivate a previous group.
    if (traveled > map1SensorMaxTicks) map1SensorMaxTicks = traveled;
    if (map1SensorMaxTicks < map1SensorStage2Ticks)
        return map1SensorStageMasks[0];
    if (map1SensorMaxTicks < map1SensorStage3Ticks)
        return map1SensorStageMasks[1];
    return map1SensorStageMasks[2];
}

// Compute only the PID measurement from enabled sensors. The original
// SENSOR_WEIGHTS coordinates and physical centre S7/S8 remain unchanged.
// No subset renumbering, gain change, PID reset, or ADC read suppression.
static bool calculateMaskedPidPosition(uint16_t allowedMask, float *outPosition) {
    if (!outPosition || !calibrationReady) return false;
    const uint16_t visible = sensor_get_line_bits() & allowedMask;
    float weighted = 0.0f;
    uint32_t strength = 0;
    for (uint8_t i = 0; i < N_SENSORS; ++i) {
        if (!(visible & (uint16_t(1U) << i))) continue;
        const uint16_t calibrated = sensor_get_calibrated(i);
        weighted += (float)calibrated * SENSOR_WEIGHTS[i];
        strength += calibrated;
    }
    if (strength == 0) return false; // Do not fabricate PID evidence.
    *outPosition = weighted / (float)strength;
    return true;
}
constexpr float OUTER_SENSOR_SPEED_REDUCTION = 0.40f;

// A sharp turn is entered only when the centroid reaches the outer region.
// If the line is lost, AUTO pivots in place to search for it.
constexpr float SHARP_TURN_POSITION = 40.0f;          // was 43 — enter turn earlier
constexpr float SHARP_TURN_POSITION_WITH_OUTER_EYES = 28.0f; // was 31
constexpr uint32_t SHARP_TURN_PROFILE_MS = 700;
constexpr float TURN_CENTERED_POSITION = 22.0f;
constexpr uint8_t TURN_CENTERED_FRAMES = 2;
constexpr uint16_t TURN_CENTER_SENSOR_MASK = 0x03C0U;
constexpr uint8_t TURN_CENTER_SENSOR_MIN = 2;
constexpr float TURN_CURVE_START_POSITION = 28.0f;
constexpr float TURN_CURVE_END_POSITION = 75.0f;
constexpr float TURN_CURVE_INNER_FACTOR_OPEN = 0.75f;
constexpr float TURN_CURVE_INNER_FACTOR_TIGHT = 0.35f;

constexpr float SPEED_GOVERNOR_FULL_ERROR = 30.0f;
constexpr float SPEED_GOVERNOR_REDUCTION = 0.45f;

// Temporarily disable encoder wheel PI. Keep its code for future experiments.
constexpr bool WHEEL_PID_ENABLED = false;
// Cascaded control: line PID produces SIGNED wheel speed setpoints; wheel PI
// closes the speed loop with quadrature encoder feedback. Tune on the robot:
// one PWM unit corresponds to approximately 6 wheel encoder ticks/second.
// This factor and gains are initial estimates, NOT calibrated motor constants.
constexpr float WHEEL_TPS_PER_PWM = 6.0f;
constexpr uint32_t WHEEL_SPEED_SAMPLE_US = 20000U;  // 50 Hz speed loop
constexpr float WHEEL_SPEED_FILTER_ALPHA = 0.4f;
constexpr float WHEEL_KP = 0.045f;  // PWM / (ticks/s)
constexpr float WHEEL_KI = 0.13f;   // PWM / (ticks)
constexpr float WHEEL_I_LIMIT_PWM = 35.0f;
constexpr float WHEEL_CORRECTION_LIMIT_PWM = 55.0f;
float wheelSpeedTps[2] = {0.0f, 0.0f};
float wheelIntegralPwm[2] = {0.0f, 0.0f};
float wheelCorrectionPwm[2] = {0.0f, 0.0f};
int32_t wheelPrevEnc[2] = {0, 0};
uint32_t wheelPrevSampleUs = 0;
bool wheelFeedbackReady = false;
int8_t wheelPrevDirection[2] = {0, 0};
constexpr float OUTER_STEERING_START_ERROR = 20.0f;
constexpr float OUTER_STEERING_FULL_ERROR = 60.0f;

enum TurnState : uint8_t {
    TURN_NONE,
    TURN_LEFT,
    TURN_RIGHT,
    SEARCH_LEFT,
    SEARCH_RIGHT
};
TurnState turnState = TURN_NONE;
TurnState lastTurnDirection = TURN_RIGHT;
uint32_t turnStartedMs = 0;
uint32_t searchStartedMs = 0;
bool earlyTurnActive = false;
bool edgeLeftSeeded = false, edgeRightSeeded = false;
uint32_t edgeLeftSeedMs = 0, edgeRightSeedMs = 0;
float edgeLeftSeedCentroid = 0.0f, edgeRightSeedCentroid = 0.0f;
uint8_t edgeLeftFrames = 0, edgeRightFrames = 0;
uint16_t edgeTrackSensorMask = 0;
bool edgeRecentCenterLeft = false, edgeRecentCenterRight = false;
uint32_t edgeRecentCenterLeftMs = 0, edgeRecentCenterRightMs = 0;
uint8_t directLeftEdgeFrames = 0, directRightEdgeFrames = 0;
uint32_t autoStartedMs = 0;
uint32_t turnMissingSinceMs = 0;
uint8_t centeredTurnFrames = 0;
uint8_t derivativeWarmupFrames = 0;
uint8_t reacquireSpeedFrames = 0;
bool turnMissing = false;
bool startupLossActive = false;
bool finishLineLatched = false;
bool finishDetectionArmed = false;
bool startPadExitCandidate = false;
bool startPadDepartureSeen = false;
uint32_t startPadDepartureMs = 0;
uint32_t startPadExitSinceMs = 0;
uint8_t fullLineFrames = 0;

bool lineLocked = false, previouslyLocked = false;
bool acquiring = false, missing = false;
uint32_t acquireSinceMs = 0, missingSinceMs = 0;
uint32_t confirmedLosses = 0, reacquisitions = 0;
float governedForwardSpeed = 0.0f;

static int16_t clampPwm(float value) {
    if (value <= 0.0f) return 0;
    if (value >= pidConfig.maxSpeed) return pidConfig.maxSpeed;
    return (int16_t)value;
}
static int16_t clampPwmSigned(float value) {
    if (value >= pidConfig.maxSpeed) return pidConfig.maxSpeed;
    if (value <= -pidConfig.maxSpeed) return -pidConfig.maxSpeed;
    return (int16_t)value;
}
static uint8_t bitCount16(uint16_t value) {
    return (uint8_t)__builtin_popcount((unsigned)value);
}
static bool hasAdjacentLineSensors(uint16_t mask) {
    return (mask & (uint16_t)(mask >> 1)) != 0;
}
static void resetWheelSpeedPid() {
    wheelFeedbackReady = false;
    wheelPrevSampleUs = 0;
    for (uint8_t i = 0; i < 2; ++i) {
        wheelSpeedTps[i] = 0.0f;
        wheelIntegralPwm[i] = 0.0f;
        wheelCorrectionPwm[i] = 0.0f;
        wheelPrevDirection[i] = 0;
    }
}

// Called every line-controller cycle, wheel speed is refreshed at 50 Hz.
// Feed-forward preserves the original line PID wheel command; encoder feedback
// compensates loading and wheel mismatch without changing the line PID gains.
static void driveWheelSpeedCascade(int16_t nominalLeftPwm,
                                   int16_t nominalRightPwm) {
    int32_t encL = 0, encR = 0;
    encoder_get_counts(&encL, &encR);
    const int32_t currentEnc[2] = {encL, encR};
    const int16_t nominalPwm[2] = {nominalLeftPwm, nominalRightPwm};
    const uint32_t nowUs = micros();

    if (!wheelFeedbackReady) {
        wheelPrevEnc[0] = encL;
        wheelPrevEnc[1] = encR;
        wheelPrevSampleUs = nowUs;
        wheelFeedbackReady = true;
    }

    const uint32_t elapsedUs = nowUs - wheelPrevSampleUs;
    if (elapsedUs >= WHEEL_SPEED_SAMPLE_US) {
        const float dt = (float)elapsedUs * 1.0e-6f;
        const bool sampleValid = dt >= 0.015f && dt <= 0.1f;
        for (uint8_t i = 0; i < 2; ++i) {
            const int32_t delta = currentEnc[i] - wheelPrevEnc[i];
            if (sampleValid) {
                const float measured = (float)delta / dt;
                wheelSpeedTps[i] += WHEEL_SPEED_FILTER_ALPHA *
                    (measured - wheelSpeedTps[i]);
                const float target = (float)nominalPwm[i] * WHEEL_TPS_PER_PWM;
                const float error = target - wheelSpeedTps[i];
                const float proposedI = fmaxf(-WHEEL_I_LIMIT_PWM,
                    fminf(WHEEL_I_LIMIT_PWM,
                          wheelIntegralPwm[i] + WHEEL_KI * error * dt));
                const float proposedCorrection = WHEEL_KP * error + proposedI;
                // Do not integrate further into the correction saturation.
                if (fabsf(proposedCorrection) < WHEEL_CORRECTION_LIMIT_PWM ||
                    (proposedCorrection > 0 && error < 0) ||
                    (proposedCorrection < 0 && error > 0)) {
                    wheelIntegralPwm[i] = proposedI;
                }
                wheelCorrectionPwm[i] = fmaxf(-WHEEL_CORRECTION_LIMIT_PWM,
                    fminf(WHEEL_CORRECTION_LIMIT_PWM,
                          WHEEL_KP * error + wheelIntegralPwm[i]));
            } else {
                wheelSpeedTps[i] = 0.0f;
                wheelIntegralPwm[i] = 0.0f;
                wheelCorrectionPwm[i] = 0.0f;
            }
            wheelPrevEnc[i] = currentEnc[i];
        }
        wheelPrevSampleUs = nowUs;
    }

    int16_t drivePwm[2] = {0, 0};
    for (uint8_t i = 0; i < 2; ++i) {
        const int8_t direction = nominalPwm[i] > 0 ? 1 :
                                nominalPwm[i] < 0 ? -1 : 0;
        if (!direction || (wheelPrevDirection[i] &&
                           wheelPrevDirection[i] != direction)) {
            wheelIntegralPwm[i] = 0.0f;
            wheelCorrectionPwm[i] = 0.0f;
        }
        wheelPrevDirection[i] = direction;
        if (!direction) continue;
        const float requested = (float)nominalPwm[i] + wheelCorrectionPwm[i];
        // Prevent wheel PI from accidentally reversing the line PID's command.
        const float aligned = direction > 0 ? fmaxf(0.0f, requested) :
                                             fminf(0.0f, requested);
        drivePwm[i] = clampPwmSigned(aligned);
    }
    motor_set_speeds(drivePwm[0], drivePwm[1]);
}

static void clearPidState() {
    lastError = integralError = lastCorrection = 0;
    lastPTerm = lastITerm = lastDTerm = 0;
    derivativeHistoryValid = false;
    derivativeWarmupFrames = 0;
}
static void resetLineTracking() {
    resetWheelSpeedPid();
    lineLocked = previouslyLocked = acquiring = missing = false;
    acquireSinceMs = missingSinceMs = 0;
    lastKnownPosition = 0;
    turnState = TURN_NONE;
    lastTurnDirection = TURN_RIGHT;
    turnStartedMs = 0;
    searchStartedMs = 0;
    earlyTurnActive = false;
    edgeLeftSeeded = edgeRightSeeded = false;
    edgeLeftFrames = edgeRightFrames = 0;
    edgeTrackSensorMask = 0;
    edgeRecentCenterLeft = edgeRecentCenterRight = false;
    edgeRecentCenterLeftMs = edgeRecentCenterRightMs = 0;
    directLeftEdgeFrames = directRightEdgeFrames = 0;
    turnMissingSinceMs = 0;
    centeredTurnFrames = 0;
    derivativeWarmupFrames = 0;
    reacquireSpeedFrames = 0;
    turnMissing = false;
    startupLossActive = false;
    finishLineLatched = false;
    finishDetectionArmed = false;
    startPadExitCandidate = false;
    startPadDepartureSeen = false;
    startPadDepartureMs = 0;
    startPadExitSinceMs = 0;
    fullLineFrames = 0;
    lastDashedCorrection = 0;
    governedForwardSpeed = 0.0f;
    gapActive = false;
    gapStartMs = 0;
    gapHoldPwm = lastNormalForwardPwm = 0;
    highSpeedStage = HS_DISABLED;
    highSpeedAborted = false;
    highSpeedMaxProgress = 0;
    map1MaxProgress = 0;
    map1LateBoostEngaged = false;
    map1SensorMaxTicks = 0;
    map2MaxProgress = 0;
    map2SensorMaxTravel = 0;
    map2SensorActiveStage = 0;
    map2HoldState = 0;
     map1Route.begin(autoMode && selectedMap == 1 && map1RouteLockEnabled);
}

// Sharp turn: outer wheel PWM adapts to how long we've been turning.
// Start faster, ramp down as timeout approaches to avoid overshooting.
static int16_t sharpTurnPwm() {
    const uint32_t elapsed = (uint32_t)(millis() - turnStartedMs);
    // First 200 ms: full power.  Then linearly taper to 70% by timeout.
    float frac = 1.0f;
    if (elapsed > 200) {
        frac = 1.0f - 0.3f * (float)(elapsed - 200) / (float)(SHARP_TURN_PROFILE_MS - 200);
        if (frac < 0.7f) frac = 0.7f;
    }
    // No extra 100..185 PWM clamp: only the configured motor 8-bit ceiling.
    return clampPwm((float)(pidConfig.baseSpeed + 40) * frac);
}
static int16_t sharpTurnInnerPwm(int16_t outer, float position) {
    float severity = (fabsf(position) - TURN_CURVE_START_POSITION) /
                     (TURN_CURVE_END_POSITION - TURN_CURVE_START_POSITION);
    if (severity < 0.0f) severity = 0.0f;
    if (severity > 1.0f) severity = 1.0f;
    const float factor = TURN_CURVE_INNER_FACTOR_OPEN -
        severity * (TURN_CURVE_INNER_FACTOR_OPEN - TURN_CURVE_INNER_FACTOR_TIGHT);
    return (int16_t)((float)outer * factor);
}
// Detect the center-to-edge trend using ONLY the currently enabled PID
// eyes. Physical all-16 sensing remains unchanged for START and FINISH.
// Clear unfinished trends whenever the encoder-selected mask stage changes.
static int8_t detectEarlyEdgeTurn(uint16_t physicalMask, uint32_t now) {
    const uint16_t allowedMask = activePidSensorMask();
    if (allowedMask != edgeTrackSensorMask) {
        edgeTrackSensorMask = allowedMask;
        edgeLeftSeeded = edgeRightSeeded = false;
        edgeLeftFrames = edgeRightFrames = 0;
        edgeRecentCenterLeft = edgeRecentCenterRight = false;
        directLeftEdgeFrames = directRightEdgeFrames = 0;
    }
    const uint16_t mask = physicalMask & allowedMask;
    const uint8_t blackCount = bitCount16(mask);
    const int32_t turnProgress = selectedMap == 1 ? map1MaxProgress : map2MaxProgress;
    if ((selectedMap != 1 && selectedMap != 2) ||
        turnProgress < EARLY_TURN_MIN_TICKS ||
        turnProgress >= EARLY_TURN_MAX_TICKS ||
        !finishDetectionArmed || earlyTurnActive || blackCount == 0) {
        edgeLeftSeeded = edgeRightSeeded = false;
        edgeLeftFrames = edgeRightFrames = 0;
        edgeRecentCenterLeft = edgeRecentCenterRight = false;
        directLeftEdgeFrames = directRightEdgeFrames = 0;
        return 0;
    }

    const bool oppositeEdges =
        (mask & 0x000FU) != 0U && (mask & 0xF000U) != 0U;
    // Store recent central evidence even when a wider line makes the
    // original centroid-trend detector ineligible. Always obey allowedMask.
    if (!oppositeEdges && (mask & 0x01F0U) && !(mask & 0x000FU)) {
        edgeRecentCenterLeft = true;
        edgeRecentCenterLeftMs = now;
    }
    if (!oppositeEdges && (mask & 0x0F80U) && !(mask & 0xF000U)) {
        edgeRecentCenterRight = true;
        edgeRecentCenterRightMs = now;
    }
    if (edgeRecentCenterLeft &&
        (uint32_t)(now - edgeRecentCenterLeftMs) > EARLY_TURN_TREND_WINDOW_MS)
        edgeRecentCenterLeft = false;
    if (edgeRecentCenterRight &&
        (uint32_t)(now - edgeRecentCenterRightMs) > EARLY_TURN_TREND_WINDOW_MS)
        edgeRecentCenterRight = false;

    // DIRECT EDGE: only adjacent outer eyes remain black, for two
    // consecutive samples. Other enabled eyes must be white.
    const bool isolatedLeft =
        !oppositeEdges && edgeRecentCenterLeft &&
        (mask & 0xFFF0U) == 0U &&
        bitCount16(mask & 0x000FU) >= 2 &&
        hasAdjacentLineSensors(mask);
    const bool isolatedRight =
        !oppositeEdges && edgeRecentCenterRight &&
        (mask & 0x0FFFU) == 0U &&
        bitCount16(mask & 0xF000U) >= 2 &&
        hasAdjacentLineSensors(mask);
    directLeftEdgeFrames = isolatedLeft
        ? (uint8_t)(directLeftEdgeFrames < EARLY_TURN_DIRECT_EDGE_FRAMES
            ? directLeftEdgeFrames + 1 : EARLY_TURN_DIRECT_EDGE_FRAMES) : 0;
    directRightEdgeFrames = isolatedRight
        ? (uint8_t)(directRightEdgeFrames < EARLY_TURN_DIRECT_EDGE_FRAMES
            ? directRightEdgeFrames + 1 : EARLY_TURN_DIRECT_EDGE_FRAMES) : 0;
    if (directLeftEdgeFrames >= EARLY_TURN_DIRECT_EDGE_FRAMES) return -1;
    if (directRightEdgeFrames >= EARLY_TURN_DIRECT_EDGE_FRAMES) return 1;

    // Centroid trend remains the existing primary detection method.
    if (blackCount > 6 || oppositeEdges) {
        edgeLeftSeeded = edgeRightSeeded = false;
        edgeLeftFrames = edgeRightFrames = 0;
        return 0;
    }
    float indexSum = 0.0f;
    for (uint8_t i = 0; i < 16; ++i)
        if (mask & (uint16_t(1U) << i)) indexSum += (float)i;
    const float centroid = indexSum / (float)blackCount;

    if (edgeLeftSeeded &&
        (uint32_t)(now - edgeLeftSeedMs) > EARLY_TURN_TREND_WINDOW_MS) {
        edgeLeftSeeded = false;
        edgeLeftFrames = 0;
    }
    if (edgeRightSeeded &&
        (uint32_t)(now - edgeRightSeedMs) > EARLY_TURN_TREND_WINDOW_MS) {
        edgeRightSeeded = false;
        edgeRightFrames = 0;
    }

    // S4..S8 -> S3..S0. Seed only before reaching the outermost eyes.
    if (!(mask & 0x000FU) && centroid >= 4.0f && centroid <= 8.0f &&
        (mask & 0x01F0U)) {
        edgeLeftSeeded = true;
        edgeLeftSeedMs = now;
        edgeLeftSeedCentroid = centroid;
        edgeLeftFrames = 0;
    }
    // Mirror: S7..S11 -> S12..S15.
    if (!(mask & 0xF000U) && centroid >= 7.0f && centroid <= 11.0f &&
        (mask & 0x0F80U)) {
        edgeRightSeeded = true;
        edgeRightSeedMs = now;
        edgeRightSeedCentroid = centroid;
        edgeRightFrames = 0;
    }

    const bool leftReached =
        edgeLeftSeeded && (mask & 0x000FU) &&
        centroid <= 3.5f && (edgeLeftSeedCentroid - centroid) >= 1.5f;
    const bool rightReached =
        edgeRightSeeded && (mask & 0xF000U) &&
        centroid >= 11.5f && (centroid - edgeRightSeedCentroid) >= 1.5f;
    edgeLeftFrames = leftReached
        ? (uint8_t)(edgeLeftFrames < 3 ? edgeLeftFrames + 1 : 3) : 0;
    edgeRightFrames = rightReached
        ? (uint8_t)(edgeRightFrames < 3 ? edgeRightFrames + 1 : 3) : 0;
    if (edgeLeftFrames >= EARLY_TURN_EDGE_CONFIRM_FRAMES) return -1;
    if (edgeRightFrames >= EARLY_TURN_EDGE_CONFIRM_FRAMES) return 1;
    return 0;
}
static bool isSearchTurn() {
    return turnState == SEARCH_LEFT || turnState == SEARCH_RIGHT;
}
// Keep the default SEARCH PWM for all other turns and maps.
// Use relative run encoder progress so the window is consistent across AUTO runs.
static int16_t searchPwmForCurrentTurn() {
    if (selectedMap == 1 && turnState == SEARCH_LEFT) {
        const int32_t progress = runEncoderProgress();
        if (progress >= MAP1_LEFT_SEARCH_TEST_START_ENC &&
            progress <= MAP1_LEFT_SEARCH_TEST_END_ENC) {
            return MAP1_LEFT_SEARCH_TEST_PWM;
        }
    }
    return LINE_SEARCH_PWM;
}
static bool isLeftTurn() {
    return turnState == TURN_LEFT || turnState == SEARCH_LEFT;
}
static TurnState searchStateFor(TurnState direction) {
    return direction == TURN_LEFT ? SEARCH_LEFT : SEARCH_RIGHT;
}
} // namespace

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = (baseSpeed < 0) ? 0 : (baseSpeed > 255 ? 255 : baseSpeed);
    pidConfig.minSpeed = 0;
    autoMode = false;
    selectedMap = 0;
    highSpeedPeakPwm = MAP_DEFAULT_HS_PEAK_PWM;
    currentCase = LINE_CASE_STOPPED;
    clearPidState();
    resetLineTracking();
    confirmedLosses = reacquisitions = 0;
    lastComputeTimeUs = micros();
}
void pid_set_tunings(float kp, float ki, float kd) {
    if (pidConfig.Kd != kd) derivativeHistoryValid = false;
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}
void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = speed < 0 ? 0 : (speed > 255 ? 255 : speed);
 }
void pid_set_auto_mode(bool enabled) {
    autoMode = enabled;
    currentCase = LINE_CASE_STOPPED;
    resetLineTracking();
    clearPidState();
    lastComputeTimeUs = micros();
    autoStartedMs = millis();
    if (enabled) {
        encoder_get_counts(&runEncoderOriginL, &runEncoderOriginR);
        highSpeedStage = ENCODER_SPEED_PROFILE_ENABLED && selectedMap >= 1 && selectedMap <= 3 ? HS_ACCEL : HS_DISABLED;
    }
    motor_stop(); // Never start blind; wait for stable black-line detection.
}
bool pid_is_auto_mode(void) { return autoMode; }
bool pid_set_map1_route_lock(bool enabled) {
    if (autoMode) return false;
    map1RouteLockEnabled = enabled;
    map1Route.begin(false);
    return true;
}
bool pid_get_map1_route_lock(void) { return map1RouteLockEnabled; }
uint8_t pid_get_map1_route_stage(void) { return (uint8_t)map1Route.stage; }
uint8_t pid_get_map1_route_fault_code(void) { return (uint8_t)map1Route.fault; }
uint16_t pid_get_map1_selected_line_mask(void) {
    return map1Route.selectedMask;
}
bool pid_select_map(uint8_t mapId) {
    if (autoMode || mapId < 1 || mapId > 3) return false;
    // Apply the track defaults only when changing MAP, not when the Python
    // app resends AUTO:<same map> after an intentional manual adjustment.
    if (selectedMap != mapId) {
        pidConfig.baseSpeed = MAP_DEFAULT_BASE_PWM[mapId - 1];
        highSpeedPeakPwm = MAP_DEFAULT_HS_PEAK_PWM;
    }
    selectedMap = mapId;
    return true;
}
uint8_t pid_get_selected_map(void) { return selectedMap; }

// Set the Map 1 PID sensor selection profile only while stopped.
// Physical 16-channel ADC sampling, FSM, route and finish logic are untouched.
bool pid_set_map2_sensor_profile(const int32_t *ticks, const uint16_t *masks) {
    if (autoMode || !ticks || !masks ||
        ticks[0] <= 0 || ticks[1] <= ticks[0] ||
        ticks[2] <= ticks[1] || ticks[2] > 10000000) return false;
    for (uint8_t i=0;i<4;++i) if (!masks[i]) return false;
    for (uint8_t i=0;i<3;++i) map2SensorTickEnds[i] = ticks[i];
    for (uint8_t i=0;i<4;++i) map2SensorMasks[i] = masks[i];
    return true;
}
void pid_get_map2_sensor_profile(int32_t *ticks, uint16_t *masks) {
    if (ticks) for (uint8_t i=0;i<3;++i) ticks[i]=map2SensorTickEnds[i];
    if (masks) for (uint8_t i=0;i<4;++i) masks[i]=map2SensorMasks[i];
}
uint8_t pid_get_map2_hold_state(void) { return map2HoldState; }
uint8_t pid_get_map2_sensor_stage(void) {
    if (selectedMap != 2) return 0;
    return map2SensorActiveStage + 1;
}

bool pid_set_map1_sensor_profile(int32_t stage2, int32_t stage3,
                                 uint16_t mask1, uint16_t mask2, uint16_t mask3) {
    if (autoMode || stage2 <= 0 || stage3 <= stage2 ||
        stage3 > 10000000 || !mask1 || !mask2 || !mask3) return false;
    map1SensorStage2Ticks = stage2;
    map1SensorStage3Ticks = stage3;
    map1SensorStageMasks[0] = mask1;
    map1SensorStageMasks[1] = mask2;
    map1SensorStageMasks[2] = mask3;
    return true;
}

void pid_get_map1_sensor_profile(int32_t *stage2, int32_t *stage3,
                                 uint16_t *mask1, uint16_t *mask2,
                                 uint16_t *mask3) {
    if (stage2) *stage2 = map1SensorStage2Ticks;
    if (stage3) *stage3 = map1SensorStage3Ticks;
    if (mask1) *mask1 = map1SensorStageMasks[0];
    if (mask2) *mask2 = map1SensorStageMasks[1];
    if (mask3) *mask3 = map1SensorStageMasks[2];
}
uint8_t pid_get_high_speed_stage(void) { return (uint8_t)highSpeedStage; }
int32_t pid_get_map1_late_boost_start(void) {
    return selectedMap == 1 ? MAP1_LATE_BOOST_START_TICKS : 0;
}
bool pid_is_map1_late_boost_engaged(void) { return map1LateBoostEngaged; }
bool pid_set_high_speed_peak(int16_t pwm) {
    if (autoMode || pwm < 100 || pwm > 255 || pwm < pidConfig.baseSpeed)
        return false;
    highSpeedPeakPwm = pwm;
    return true;
}
int16_t pid_get_high_speed_peak(void) { return highSpeedPeakPwm; }
int32_t pid_get_high_speed_brake_start(void) { return highSpeedBrakeStart(); }
int32_t pid_get_run_encoder_progress(void) { return runEncoderProgress(); }
void pid_get_run_encoder_origins(int32_t* left, int32_t* right) {
    if (left) *left = runEncoderOriginL;
    if (right) *right = runEncoderOriginR;
}
void pid_get_gap_window(int32_t* begin, int32_t* end) {
    const GapWindow w = selectedMap >= 1 && selectedMap <= 3
                      ? GAP_WINDOWS[selectedMap - 1] : GapWindow{0, 0};
    if (begin) *begin = w.begin;
    if (end) *end = w.end;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    if (!outPosition || !outBlackCount) return false;
    float weighted = 0;
    uint32_t strength = 0;
    uint8_t blackCount = 0;
    if (calibrationReady) {
        for (int i = 0; i < N_SENSORS; i++) {
            if (!sensor_is_black(i)) continue;
            const uint16_t cal = sensor_get_calibrated(i);
            weighted += (float)cal * SENSOR_WEIGHTS[i];
            strength += cal;
            ++blackCount;
        }
    }
    *outBlackCount = blackCount;
    const uint16_t physicalMask = sensor_get_line_bits();
    map1Route.selectedMask = physicalMask;
    if (map1Route.active && finishDetectionArmed &&
        blackCount > 1 && blackCount <= 10 && strength > 0) {
        uint16_t samples[N_SENSORS];
        for (uint8_t i=0; i<N_SENSORS; ++i)
            samples[i] = sensor_get_calibrated(i);
        float selectedPosition = 0.0f;
        uint8_t selectedCount = 0;
        if (map1Route.chooseCluster(physicalMask, samples,
                                    lastKnownPosition, map1MaxProgress,
                                    selectedPosition, selectedCount)) {
            *outPosition = selectedPosition;
            *outBlackCount = selectedCount;
            lastKnownPosition = selectedPosition;
            return true;
        }
    }
    if (blackCount > 0 && strength > 0) {
        *outPosition = weighted / (float)strength;
        lastKnownPosition = *outPosition;
        return true;
    }
    *outPosition = lastKnownPosition; // Position memory is not line evidence.
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady || !sensor_adc_ready()) return LINE_CASE_STOPPED;
    if (finishLineLatched) return LINE_CASE_STOPPED;
    const uint32_t now = millis();
    const uint16_t mask = sensor_get_line_bits();
    // Stage uses monotonic maximum forward progress, not time or raw encoder
    // sum; wheel reversal during SEARCH must not reactivate earlier zones.
    if (ENCODER_SPEED_PROFILE_ENABLED && selectedMap >= 1 && selectedMap <= 3 && !highSpeedAborted) {
        const int32_t progress = runEncoderProgress();
        if (progress > highSpeedMaxProgress) highSpeedMaxProgress = progress;
        if (isSearchTurn() && highSpeedMaxProgress < HS_BRAKE_END_ENC) {
            // An unexpected pivot before braking is complete cancels HS.
            highSpeedAborted = true;
            highSpeedStage = HS_ABORTED;
        } else if (highSpeedStage == HS_FINISHED) {
            // Completion is sticky; encoder reversal cannot restart HS.
        } else if (highSpeedMaxProgress >= HS_BRAKE_END_ENC) {
            // Release immediately when both requested conditions are met:
            // current encoder progress has passed the threshold and a valid
            // line measurement is near center. blackCount>0 prevents a false
            // release because absent-line samples only return lastKnownPosition,
            // which may still be near center and is not current line evidence.
            const bool validCenteredLine =
                blackCount > 0 && fabsf(position) <= HS_HOLD_CENTER_ERROR;
            highSpeedStage =
                (progress >= HS_HOLD_MIN_END_ENC && validCenteredLine)
                    ? HS_FINISHED : HS_CORNER_HOLD;
        } else if (highSpeedMaxProgress >= highSpeedBrakeStart()) {
            highSpeedStage = HS_BRAKE;
        } else if (highSpeedMaxProgress >= HS_RAMP_END_ENC) {
            highSpeedStage = HS_CRUISE;
        } else {
            highSpeedStage = HS_ACCEL;
        }
    }

    // START is accepted with ANY calibrated pattern: WHITE, LINE or BLACK.
    // Until the narrow centered line is confirmed, drive STRAIGHT instead
    // of steering prematurely with partial-pad edges or entering SEARCH.
    // Arm the 16/16-black FINISH detector only after confirmation.
    if (!finishDetectionArmed) {
        if (mask != 0xFFFFU && !startPadDepartureSeen) {
            startPadDepartureSeen = true;
            startPadDepartureMs = now;
        }
        const bool narrowCenteredTrack =
            blackCount > 0 && blackCount <= START_PAD_EXIT_MAX_BLACK &&
            (mask & START_PAD_CENTER_MASK) != 0U &&
            fabsf(position) <= START_PAD_CENTER_TOLERANCE;
        if (narrowCenteredTrack) {
            if (!startPadExitCandidate) {
                startPadExitCandidate = true;
                startPadExitSinceMs = now;
            } else if (uint32_t(now - startPadExitSinceMs) >= START_PAD_EXIT_CONFIRM_MS) {
                finishDetectionArmed = true;
                startPadExitCandidate = false;
                // Hand over to PID with fresh state, not a remembered START
                // pad position or a derivative spike.
                lineLocked = true;
                previouslyLocked = true;
                acquiring = missing = false;
                startupLossActive = false;
                turnState = TURN_NONE;
                clearPidState();
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
            }
        } else {
            startPadExitCandidate = false;
        }
        fullLineFrames = 0;
        if (!finishDetectionArmed) {
            // A white/off-center start may coast while finding the track,
            // but must not drive indefinitely if no valid line appears.
            if (startPadDepartureSeen &&
                uint32_t(now - startPadDepartureMs) >= STARTUP_LOSS_GRACE_MS)
                return LINE_CASE_STOPPED;
            // Both the full-black pad and broad edge patterns go straight.
            // All other patterns coast straight until the center line locks.
            return blackCount >= 11 ? LINE_CASE_CROSS_LINE
                                    : LINE_CASE_STARTUP_COAST;
        }
    }
    // FINISH is now armed: 16/16 black for three consecutive PID samples
    // stops the robot, using the existing finish logic.
    if (finishDetectionArmed && bitCount16(mask) == FULL_LINE_MIN_SENSORS) {
        if (fullLineFrames < FINISH_CONFIRM_FRAMES) ++fullLineFrames;
        if (fullLineFrames >= FINISH_CONFIRM_FRAMES) {
            finishLineLatched = true;
            turnState = TURN_NONE;
            centeredTurnFrames = 0;
            return LINE_CASE_STOPPED;
        }
    } else {
        fullLineFrames = 0;
    }

    // GAP_BRIDGE has priority over startup grace and the usual 30ms dropout.
    // One timer per continuous white segment; no reentry after its timeout.
    if (gapActive) {
        if (blackCount == 0) {
            if ((uint32_t)(now - gapStartMs) < GAP_TIMEOUT_MS)
                return LINE_CASE_GAP_BRIDGE;
            gapActive = false;
            startupLossActive = false;
            lineLocked = false;
            acquiring = missing = false;
            ++confirmedLosses;
            turnState = searchStateFor(lastKnownPosition < 0.0f ? TURN_LEFT :
                                       lastKnownPosition > 0.0f ? TURN_RIGHT :
                                       lastTurnDirection);
            searchStartedMs = now;
            centeredTurnFrames = 0;
            clearPidState();
            return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
        }
        // Line returns before 100 ms: resume existing PID without stale D term.
        gapActive = false;
        missing = false;
        derivativeHistoryValid = false;
        derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
    }

    if (ENCODER_SPEED_PROFILE_ENABLED && blackCount == 0 &&
        turnState == TURN_NONE && lineLocked && !startupLossActive &&
        selectedMap >= 1 && selectedMap <= 3) {
        const int32_t progress = runEncoderProgress();
        const GapWindow w = GAP_WINDOWS[selectedMap - 1];
        if (progress >= w.begin && progress <= w.end) {
            gapActive = true;
            gapStartMs = now;
            gapHoldPwm = lastNormalForwardPwm > 0
                         ? lastNormalForwardPwm : clampPwm(pidConfig.baseSpeed);
            return LINE_CASE_GAP_BRIDGE;
        }
    }

    if (startupLossActive) {
        if (blackCount > 0) {
            startupLossActive = false;
            missing = false;
        } else if (uint32_t(now - autoStartedMs) < STARTUP_LOSS_GRACE_MS) {
            return LINE_CASE_STARTUP_COAST;
        } else {
            startupLossActive = false;
        }
    }

    // Prefer a confirmed center-to-edge trajectory over saturated normal PID
    // from AUTO START until 12000 Map-1 progress ticks. Only enabled PID
    // sensors participate in the EARLY_TURN detection.
    // Do not interrupt an already active ordinary SEARCH pivot.
    if (!isSearchTurn() && blackCount > 0) {
        const int8_t edgeDirection = detectEarlyEdgeTurn(mask, now);
        if (edgeDirection != 0) {
            turnState = edgeDirection < 0 ? SEARCH_LEFT : SEARCH_RIGHT;
            lastTurnDirection = edgeDirection < 0 ? TURN_LEFT : TURN_RIGHT;
            earlyTurnActive = true;
            searchStartedMs = now;
            centeredTurnFrames = 0;
            turnMissing = false;
            missing = false;
            edgeLeftSeeded = edgeRightSeeded = false;
            edgeLeftFrames = edgeRightFrames = 0;
            clearPidState();
            return edgeDirection < 0 ? LINE_CASE_SHARP_LEFT
                                     : LINE_CASE_SHARP_RIGHT;
        }
    } else if (blackCount == 0) {
        edgeLeftSeeded = edgeRightSeeded = false;
        edgeLeftFrames = edgeRightFrames = 0;
    }

    // Keep PID steering while a corner remains visible; pivot only after line loss.
    if (turnState != TURN_NONE) {
        if (isSearchTurn()) {
            const uint16_t reacquireMask =
                earlyTurnActive ? (mask & activePidSensorMask()) : mask;
            const uint8_t reacquireCount = bitCount16(reacquireMask);
            float reacquirePosition = position;
            const bool reacquirePositionValid = !earlyTurnActive ||
                calculateMaskedPidPosition(activePidSensorMask(), &reacquirePosition);
            const bool reliableReacquisition = earlyTurnActive
                // In EARLY_TURN, disabled eyes must not cause PID reacquisition.
                ? (reacquireCount >= 2 && reacquireCount <= 6 &&
                   (reacquireMask & TURN_CENTER_SENSOR_MASK) != 0U &&
                   reacquirePositionValid &&
                   fabsf(reacquirePosition) <= TURN_CENTERED_POSITION &&
                   hasAdjacentLineSensors(reacquireMask))
                : ((blackCount >= 2 && hasAdjacentLineSensors(mask)) ||
                   (blackCount > 0 && fabsf(position) <= TURN_CENTERED_POSITION));
            if (reliableReacquisition) {
                if (centeredTurnFrames < SEARCH_REACQUIRE_FRAMES)
                    ++centeredTurnFrames;
            } else {
                centeredTurnFrames = 0;
            }
            if (centeredTurnFrames >= SEARCH_REACQUIRE_FRAMES) {
                earlyTurnActive = false;
                turnState = TURN_NONE;
                centeredTurnFrames = 0;
                missing = false;
                turnMissing = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                reacquireSpeedFrames = REACQUIRE_SPEED_RAMP_FRAMES;
                lineLocked = true;
                if (previouslyLocked) ++reacquisitions;
                previouslyLocked = true;
                if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;
                return LINE_CASE_NORMAL_PID;
            }
        } else {
            const bool narrowCenteredLine =
                blackCount > 0 && fabsf(position) <= TURN_CENTERED_POSITION &&
                blackCount <= 6 && (mask & 0xF00FU) == 0;
            const bool wideCenteredLine =
                bitCount16(mask & TURN_CENTER_SENSOR_MASK) >= TURN_CENTER_SENSOR_MIN;
            if (narrowCenteredLine || wideCenteredLine) {
                if (centeredTurnFrames < TURN_CENTERED_FRAMES)
                    ++centeredTurnFrames;
            } else {
                centeredTurnFrames = 0;
            }
            if (centeredTurnFrames >= TURN_CENTERED_FRAMES) {
                turnState = TURN_NONE;
                centeredTurnFrames = 0;
                missing = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES; // guard against D-spike on turn exit
                lineLocked = true;
                if (previouslyLocked) ++reacquisitions;
                previouslyLocked = true;
                if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;
                return LINE_CASE_NORMAL_PID;
            }
        }

        if (!isSearchTurn() && blackCount == 0) {
            // The last valid physical line measurement was at the far edge
            // and its direction agrees with the active sharp turn. Start
            // the existing SEARCH pivot on this FIRST all-white sample.
            // Keep the normal 30ms grace for centered dashed-line gaps.
            const bool edgeCornerLost =
                (turnState == TURN_RIGHT &&
                 lastKnownPosition >= EDGE_CORNER_IMMEDIATE_SEARCH_ERROR) ||
                (turnState == TURN_LEFT &&
                 lastKnownPosition <= -EDGE_CORNER_IMMEDIATE_SEARCH_ERROR);
            if (edgeCornerLost) {
                turnState = searchStateFor(turnState);
                searchStartedMs = now;
                centeredTurnFrames = 0;
                turnMissing = false;
                missing = false;
                clearPidState();
                return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
            }
            if (!turnMissing) {
                turnMissing = true;
                turnMissingSinceMs = now;
            } else if (uint32_t(now - turnMissingSinceMs) >= LINE_LOSS_MS) {
                turnState = searchStateFor(turnState);
                searchStartedMs = now;
                centeredTurnFrames = 0;
                turnMissing = false;
                clearPidState();
                return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
            }
        } else {
            turnMissing = false;
        }

        if (isSearchTurn() &&
            uint32_t(now - searchStartedMs) >= LINE_SEARCH_TIMEOUT_MS) {
            earlyTurnActive = false;
            turnState = TURN_NONE;
            centeredTurnFrames = 0;
            lineLocked = false;
            acquiring = missing = false;
            turnMissing = false;
            ++confirmedLosses;
            return LINE_CASE_LOST_LINE;
        }
        if (!isSearchTurn() && blackCount > 0)
            return LINE_CASE_NORMAL_PID;
        return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
    }

    if (blackCount > 0) {
        missing = false;
        if (!lineLocked) {
            if (!acquiring) {
                acquiring = true;
                acquireSinceMs = now;
            }
            if (uint32_t(now - acquireSinceMs) < LINE_ACQUIRE_MS)
                return LINE_CASE_LOST_LINE;
            lineLocked = true;
            acquiring = false;
            if (previouslyLocked) ++reacquisitions;
            previouslyLocked = true;
        }

        // Cross-line takes priority over corner classification.
        if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;

        const uint8_t leftOuter = bitCount16(mask & 0x000FU);
        const uint8_t rightOuter = bitCount16(mask & 0xF000U);
        const bool sharpLeft =
            position <= -SHARP_TURN_POSITION ||
            (position <= -SHARP_TURN_POSITION_WITH_OUTER_EYES && leftOuter >= 2);
        const bool sharpRight =
            position >= SHARP_TURN_POSITION ||
            (position >= SHARP_TURN_POSITION_WITH_OUTER_EYES && rightOuter >= 2);

        if (sharpLeft || sharpRight) {
            turnState = sharpLeft ? TURN_LEFT : TURN_RIGHT;
            lastTurnDirection = turnState;
            turnStartedMs = now;
            centeredTurnFrames = 0;
            turnMissing = false;
            clearPidState();
            return LINE_CASE_NORMAL_PID;
        }
        return LINE_CASE_NORMAL_PID;
    }

    acquiring = false;
    if (!lineLocked) return LINE_CASE_LOST_LINE;
    if (!missing) {
        missing = true;
        missingSinceMs = now;
    }
    if (uint32_t(now - missingSinceMs) < LINE_LOSS_MS)
        return LINE_CASE_DASHED_LINE;
    if (previouslyLocked &&
        uint32_t(now - autoStartedMs) < STARTUP_LOSS_GRACE_MS) {
        startupLossActive = true;
        return LINE_CASE_STARTUP_COAST;
    }
    lineLocked = false;
    missing = false;
    ++confirmedLosses;
    turnState = searchStateFor(lastKnownPosition < 0.0f ? TURN_LEFT :
                               lastKnownPosition > 0.0f ? TURN_RIGHT :
                               lastTurnDirection);
    searchStartedMs = now;
    centeredTurnFrames = 0;
    clearPidState();
    return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
}

void pid_line_follow_step(void) {
    if (!autoMode) return;
    float pos = 0;
    uint8_t blackCount = 0;
    if (selectedMap == 1) {
        const int32_t progress = runEncoderProgress();
        if (progress > map1MaxProgress) map1MaxProgress = progress;
    }
    const bool good = pid_calculate_position(&pos, &blackCount);
    // Map 2 progresses continuously; only retain ticks for early turn recognition.
    if (selectedMap == 2) {
        const int32_t progress = runEncoderProgress();
        if (progress > map2MaxProgress) map2MaxProgress = progress;
    }
    // Physical position/blackCount continue to drive all existing FSM cases.
    // Only the NORMAL_PID steering error uses the encoder-selected subset.
    const uint16_t pidSensorMask = activePidSensorMask();
    float pidPosition = pos;
    const bool pidMeasurementGood =
        pidSensorMask == MAP1_SENSOR_MASK_STAGE3
            ? good
            : calculateMaskedPidPosition(pidSensorMask, &pidPosition);
    const LineCase_t prev = currentCase;
    currentCase = pid_detect_case(pos, blackCount);
    if (map1Route.active && currentCase != LINE_CASE_STOPPED) {
        const bool validRouteLine =
            finishDetectionArmed && lineLocked &&
            blackCount > 0 && blackCount <= 10;
        map1Route.observe(map1MaxProgress, pos, validRouteLine,
                          currentCase == LINE_CASE_NORMAL_PID, millis());
        const bool white = sensor_get_line_bits() == 0U;
        map1Route.monitorFirstSearch(map1MaxProgress, isSearchTurn(),
                                     white, millis());
        // FIRST all-white gap inside the semicircle: choose RIGHT, never
        // pivot blindly based on ticks alone.
        if (map1Route.firstRightSearchWindow(map1MaxProgress) &&
            white && isSearchTurn()) {
            if (turnState != SEARCH_RIGHT) {
                turnState = SEARCH_RIGHT;
                searchStartedMs = millis();
            }
            currentCase = LINE_CASE_SHARP_RIGHT;
        }
    }
    if (map1Route.stage == map1route::FAULT) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
        return;
    }
    const uint32_t nowUs = micros();
    const float measuredDt = (float)(nowUs - lastComputeTimeUs) * 1e-6f;
    const bool dtValid = measuredDt > 0 && measuredDt <= 0.1f;
    const float dt = dtValid ? measuredDt : 0.01f;
    lastComputeTimeUs = nowUs;
    if (prev != LINE_CASE_NORMAL_PID || currentCase != LINE_CASE_NORMAL_PID)
        derivativeHistoryValid = false;

    // Close wheel speed loop only when the line PID has a valid line measurement.
    // SEARCH, dash gaps, crossing and stop keep their established direct PWM logic.
    if (currentCase != LINE_CASE_NORMAL_PID || !pidMeasurementGood)
        resetWheelSpeedPid();
    if (currentCase == LINE_CASE_SHARP_LEFT) {
        clearPidState();
        if (turnState == SEARCH_LEFT) {
            if (earlyTurnActive)
                motor_set_speeds(-EARLY_TURN_INNER_REVERSE_PWM,
                                  EARLY_TURN_OUTER_FORWARD_PWM);
            else {
                const int16_t searchPwm = searchPwmForCurrentTurn();
                motor_set_speeds(-searchPwm, searchPwm);
            }
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(inner, outer);
        }
    } else if (currentCase == LINE_CASE_SHARP_RIGHT) {
        clearPidState();
        if (turnState == SEARCH_RIGHT) {
            if (earlyTurnActive)
                motor_set_speeds(EARLY_TURN_OUTER_FORWARD_PWM,
                                 -EARLY_TURN_INNER_REVERSE_PWM);
            else
                motor_set_speeds(LINE_SEARCH_PWM, -LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(outer, inner);
        }
    } else if (currentCase == LINE_CASE_NORMAL_PID && pidMeasurementGood) {
        const float err = pidPosition;
        const float absErr = fabsf(err);

        // Non-linear P gain: ramps up quadratically as error increases.
        // This makes the robot react gently near center but aggressively near edges.
        const float p = pidConfig.Kp * (1.0f + 0.018f * absErr) * err;

        // Integral with anti-windup: tighter clamp + decay when error crosses zero
        if ((integralError > 0 && err < 0) || (integralError < 0 && err > 0))
            integralError *= 0.5f;  // fast decay on zero-crossing to avoid overshoot
        integralError += err * dt;
        if (integralError > 80.0f) integralError = 80.0f;
        if (integralError < -80.0f) integralError = -80.0f;
        const float it = pidConfig.Ki * integralError;

        // Derivative on error with low-pass filter to reduce noise spikes
        float d = 0.0f;
        if (derivativeWarmupFrames > 0) {
            --derivativeWarmupFrames;
        } else if (pidConfig.Kd != 0 && derivativeHistoryValid && dtValid) {
            const float rawD = pidConfig.Kd * (err - lastError) / measuredDt;
            // First-order low-pass: alpha=0.6 balances responsiveness vs noise
            d = 0.6f * rawD + 0.4f * lastDTerm;
            // Hard clamp: prevents catastrophic D-spikes when re-entering PID after a turn
            constexpr float D_CLAMP = 200.0f;
            if (d >  D_CLAMP) d =  D_CLAMP;
            if (d < -D_CLAMP) d = -D_CLAMP;
        }
        lastError = err;
        derivativeHistoryValid = true;
        lastPTerm = p;
        lastITerm = it;
        lastDTerm = d;
        lastCorrection = p + it + d;

        // Map 1 only: once 10,500 forward-progress ticks have been reached,
        // boost the base PWM to HS_PEAK on a confirmed centered line.
        // The boost remains armed until the next AUTO start, but is applied
        // only while the line is centered; corners retain the configured BASE.
        // Maps 2 and 3 always keep the regular BASE.
        const bool map1ReadyForBoost = selectedMap == 1 &&
            map1MaxProgress >= MAP1_LATE_BOOST_START_TICKS;
        const bool centeredForBoost = pidMeasurementGood &&
            fabsf(pidPosition) <= MAP1_LATE_BOOST_CENTER_ERROR;
        if (map1ReadyForBoost && centeredForBoost)
            map1LateBoostEngaged = true;
        float base = (float)pidConfig.baseSpeed;
        if (map1ReadyForBoost && centeredForBoost)
            base = (float)highSpeedPeakPwm;
        governedForwardSpeed = base;
        const float correction = lastCorrection;

        // Preserve the current governed forward PWM for map-specific gaps.
        lastNormalForwardPwm = clampPwm(base);
        // Save correction for dashed-line coast
        lastDashedCorrection = correction;

        float leftCommand = base + correction;
        float rightCommand = base - correction;
        // With wheel PI disabled, output the line PID's signed PWM directly.
        // The wheel-speed feedback implementation remains available for re-enable.
        const int16_t leftPwm = clampPwmSigned(leftCommand);
        const int16_t rightPwm = clampPwmSigned(rightCommand);
        if (WHEEL_PID_ENABLED) {
            driveWheelSpeedCascade(leftPwm, rightPwm);
        } else {
            motor_set_speeds(leftPwm, rightPwm);
        }
    } else if (currentCase == LINE_CASE_CROSS_LINE) {
        clearPidState();
        // On the all-black START pad, use BASE to ensure forward motion.
        // Other wide intersections retain the original 70% BASE crossing.
        const bool initialBlackPad =
            !finishDetectionArmed && sensor_get_line_bits() == 0xFFFFU;
        const float crossingFactor = initialBlackPad ? 1.0f : 0.7f;
        const int16_t speed = clampPwm((float)pidConfig.baseSpeed * crossingFactor);
        motor_set_speeds(speed, speed);
    } else if (currentCase == LINE_CASE_GAP_BRIDGE) {
        motor_set_speeds(gapHoldPwm, gapHoldPwm);
    } else if (currentCase == LINE_CASE_DASHED_LINE) {
        // Coast at elevated speed through brief line gaps (dashed line / intersection).
        // Use a heavily-damped correction so the robot goes mostly straight instead of
        // following the last turn direction, which causes sideways stutter.
        const float coastBase = fminf(governedForwardSpeed,
                                      (float)DROPOUT_FORWARD_PWM);
        const float coastCorrection = lastDashedCorrection * 0.3f; // was 0.7f — less sideways drift
        float leftCommand = coastBase + coastCorrection;
        float rightCommand = coastBase - coastCorrection;
        motor_set_speeds(clampPwmSigned(leftCommand),
                         clampPwmSigned(rightCommand));
    } else if (currentCase == LINE_CASE_STARTUP_COAST) {
        const int16_t speed = pidConfig.baseSpeed < DROPOUT_FORWARD_PWM
            ? pidConfig.baseSpeed : DROPOUT_FORWARD_PWM;
        motor_set_speeds(speed, speed);
    } else {
        clearPidState();
        motor_stop(); // Safe stop after confirmed line loss or any sensor fault.
    }
}

LineCase_t pid_get_current_case(void) { return currentCase; }
bool pid_line_is_confirmed(void) { return lineLocked; }
bool pid_finish_line_latched(void) { return finishLineLatched; }
bool pid_finish_detection_armed(void) { return finishDetectionArmed; }
uint32_t pid_get_confirmed_line_loss_count(void) { return confirmedLosses; }
uint32_t pid_get_line_reacquisition_count(void) { return reacquisitions; }

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line PID";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line - STOP";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Nga Tu";
        case LINE_CASE_T_LEFT: return "CASE 6: Nga 3 Trai";
        case LINE_CASE_T_RIGHT: return "CASE 7: Nga 3 Phai";
        case LINE_CASE_STOPPED: return "CASE 8: Dung Xe";
        case LINE_CASE_DASHED_LINE: return "CASE 9: Pending loss - forward only";
        case LINE_CASE_STARTUP_COAST: return "CASE 10: Startup loss - straight";
        case LINE_CASE_GAP_BRIDGE: return "CASE 11: Map GAP bridge 100ms";
        case LINE_CASE_MAP2_HOLD: return "CASE 12: Map2 stationary line hold";
        default: return "Khong Xac Dinh";
    }
}
float pid_get_last_error(void) { return lastError; }
float pid_get_correction(void) { return lastCorrection; }
float pid_get_term_p(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastPTerm : 0.0f; }
float pid_get_term_i(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastITerm : 0.0f; }
float pid_get_term_d(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastDTerm : 0.0f; }
float pid_get_kp(void) { return pidConfig.Kp; }
float pid_get_ki(void) { return pidConfig.Ki; }
float pid_get_kd(void) { return pidConfig.Kd; }
int16_t pid_get_base_speed(void) { return pidConfig.baseSpeed; }
