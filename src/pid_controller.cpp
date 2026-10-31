#include "pid_controller.h"
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
static PID_Config_t pidConfig = {3.8f, 0.015f, 2.0f, 100, 255, -100};
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
constexpr uint32_t LINE_LOSS_MS = 40;           // was 25 — longer coast before declaring lost
constexpr uint32_t LINE_SEARCH_TIMEOUT_MS = 2000;
constexpr uint32_t STARTUP_LOSS_GRACE_MS = 5000;
// First AUTO frame on a completely black start box: forward-only escape.
// Stops rather than drives indefinitely if no narrow line is found.
constexpr uint32_t START_BOX_TIMEOUT_MS = 2500;
// Minimum straight escape drive; independent from low NORMAL_PID base.
// Tune this against loaded wheel motion before running at full speed.
constexpr int16_t START_BOX_PWM = 135;
constexpr uint8_t START_BOX_LINE_FRAMES = 3;
constexpr uint32_t START_BOX_LINE_STABLE_MS = 20;
constexpr int16_t DROPOUT_FORWARD_PWM = 65;
constexpr int16_t LINE_SEARCH_PWM = 85;
// Require a continuous, centered and contiguous line before exiting SEARCH.
constexpr uint32_t SEARCH_REACQUIRE_CONFIRM_MS = 40;
constexpr uint32_t REACQUIRE_SETTLE_MS = 150;
constexpr uint32_t REACQUIRE_INVALID_MS = 40;
// Do not reset this watchdog when a false reacquisition briefly leaves SEARCH.
constexpr uint32_t RECOVERY_SESSION_TIMEOUT_MS = 2500;
constexpr uint8_t REACQUIRE_DERIVATIVE_HOLD_FRAMES = 4;
constexpr float REACQUIRE_CORRECTION_MULTIPLIER = 1.0f;
constexpr float SEARCH_CANDIDATE_PWM = 95.0f;
constexpr float SEARCH_CANDIDATE_KP = 0.65f;
constexpr float SEARCH_CANDIDATE_MAX_CORRECTION = 20.0f;
// Increase only NORMAL_PID forward reference by 10%; other modes unchanged.
constexpr float NORMAL_PID_BASE_GAIN = 1.10f;
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
constexpr int16_t TURN_PWM_MIN = 100;           // was 95 — slightly more aggressive
constexpr int16_t TURN_PWM_MAX = 185;           // was 170 — allow faster outer wheel
constexpr float TURN_CURVE_START_POSITION = 28.0f;
constexpr float TURN_CURVE_END_POSITION = 75.0f;
constexpr float TURN_CURVE_INNER_FACTOR_OPEN = 0.75f;
constexpr float TURN_CURVE_INNER_FACTOR_TIGHT = 0.35f;

constexpr float SPEED_GOVERNOR_FULL_ERROR = 30.0f;
constexpr float SPEED_GOVERNOR_REDUCTION = 0.45f;
constexpr float PID_CORRECTION_BASE_MULTIPLIER = 2.0f;
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
uint32_t autoStartedMs = 0;
uint32_t turnMissingSinceMs = 0;
uint8_t centeredTurnFrames = 0;
uint8_t derivativeWarmupFrames = 0;
bool searchCandidateActive = false;
uint32_t searchCandidateSinceMs = 0;
bool settlingActive = false;
uint32_t settleStartedMs = 0;
uint32_t settleInvalidSinceMs = 0;
bool recoverySessionActive = false;
uint32_t recoveryStartedMs = 0;
bool recoveryTimeoutLatched = false;
bool turnMissing = false;
bool startupLossActive = false;
bool startBoxCheckPending = false;
bool startBoxEscaping = false;
bool startBoxTimedOut = false;
uint32_t startBoxStartedMs = 0;
uint8_t startBoxLineFrames = 0;
uint32_t startBoxLineSinceMs = 0;
bool fullLineCandidate = false;
bool finishLineLatched = false;
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
static float outerSensorSeverity(uint16_t mask) {
    float severity = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
        if (mask & (uint16_t)(1U << i)) {
            const float sensorSeverity = (float)(4U - i) * 0.25f;
            if (sensorSeverity > severity) severity = sensorSeverity;
        }
        const uint8_t rightSensor = (uint8_t)(15U - i);
        if (mask & (uint16_t)(1U << rightSensor)) {
            const float sensorSeverity = (float)(4U - i) * 0.25f;
            if (sensorSeverity > severity) severity = sensorSeverity;
        }
    }
    return severity;
}
static bool hasAdjacentLineSensors(uint16_t mask) {
    return (mask & (uint16_t)(mask >> 1)) != 0;
}
// A centered narrow line must be a SINGLE contiguous band, not separate black
// islands or a wide crossing. Allow one black eye when it is near center.
static bool isCenteredNarrowLine(uint16_t mask, float position, uint8_t blackCount) {
    if (blackCount == 0 || blackCount > 6 ||
        fabsf(position) > TURN_CENTERED_POSITION ||
        (mask & TURN_CENTER_SENSOR_MASK) == 0) return false;
    const uint16_t shifted = (uint16_t)(mask >> __builtin_ctz((unsigned)mask));
    return (shifted & (uint16_t)(shifted + 1U)) == 0;
}
// Forward, bounded steering keeps the candidate line underneath the array
// while validating it. Stopping the motors here stranded the robot in SEARCH.
static void driveSearchCandidate(float position) {
    const float correction = fmaxf(-SEARCH_CANDIDATE_MAX_CORRECTION,
        fminf(SEARCH_CANDIDATE_KP * position,
              SEARCH_CANDIDATE_MAX_CORRECTION));
    motor_set_speeds((int16_t)(SEARCH_CANDIDATE_PWM + correction),
                     (int16_t)(SEARCH_CANDIDATE_PWM - correction));
}
static void limitMotorCommandPeak(float& left, float& right, float limit) {
    const float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > limit && peak > 0.0f) {
        const float scale = limit / peak;
        left *= scale;
        right *= scale;
    }
}
static float updateForwardSpeed(float absErr, float referenceSpeed) {
    const float errorRatio = fminf(absErr / SPEED_GOVERNOR_FULL_ERROR, 1.0f);
    float targetSpeed = referenceSpeed *
        (1.0f - SPEED_GOVERNOR_REDUCTION * errorRatio);
    const float minimumSpeed = fminf(referenceSpeed, 45.0f);
    if (targetSpeed < minimumSpeed) targetSpeed = minimumSpeed;
    if (targetSpeed > pidConfig.maxSpeed) targetSpeed = pidConfig.maxSpeed;
    governedForwardSpeed = targetSpeed;
    return governedForwardSpeed;
}
static void clearPidState() {
    lastError = integralError = lastCorrection = 0;
    lastPTerm = lastITerm = lastDTerm = 0;
    derivativeHistoryValid = false;
    derivativeWarmupFrames = 0;
}
static void resetLineTracking() {
    lineLocked = previouslyLocked = acquiring = missing = false;
    acquireSinceMs = missingSinceMs = 0;
    lastKnownPosition = 0;
    turnState = TURN_NONE;
    lastTurnDirection = TURN_RIGHT;
    turnStartedMs = 0;
    searchStartedMs = 0;
    turnMissingSinceMs = 0;
    centeredTurnFrames = 0;
    derivativeWarmupFrames = 0;
    searchCandidateActive = false;
    searchCandidateSinceMs = 0;
    settlingActive = false;
    settleStartedMs = 0;
    settleInvalidSinceMs = 0;
    recoverySessionActive = false;
    recoveryStartedMs = 0;
    recoveryTimeoutLatched = false;
    turnMissing = false;
    startupLossActive = false;
    startBoxCheckPending = false;
    startBoxEscaping = false;
    startBoxTimedOut = false;
    startBoxStartedMs = 0;
    startBoxLineFrames = 0;
    startBoxLineSinceMs = 0;
    fullLineCandidate = false;
    finishLineLatched = false;
    fullLineFrames = 0;
    lastDashedCorrection = 0;
    governedForwardSpeed = 0.0f;
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
    int32_t speed = (int32_t)((float)(pidConfig.baseSpeed + 40) * frac);
    if (speed < TURN_PWM_MIN) speed = TURN_PWM_MIN;
    if (speed > TURN_PWM_MAX) speed = TURN_PWM_MAX;
    return (int16_t)speed;
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
static bool isSearchTurn() {
    return turnState == SEARCH_LEFT || turnState == SEARCH_RIGHT;
}
static bool isLeftTurn() {
    return turnState == TURN_LEFT || turnState == SEARCH_LEFT;
}
static TurnState searchStateFor(TurnState direction) {
    return direction == TURN_LEFT ? SEARCH_LEFT : SEARCH_RIGHT;
}
static void beginRecoverySearch(TurnState direction, uint32_t now) {
    if (!recoverySessionActive) {
        recoverySessionActive = true;
        recoveryStartedMs = now;
    }
    ++confirmedLosses;
    turnState = searchStateFor(direction);
    searchStartedMs = now;
    searchCandidateActive = false;
    searchCandidateSinceMs = 0;
    settlingActive = false;
    settleInvalidSinceMs = 0;
    centeredTurnFrames = 0;
    turnMissing = false;
    missing = false;
}
static void latchRecoveryTimeout() {
    recoveryTimeoutLatched = true;
    recoverySessionActive = false;
    settlingActive = false;
    searchCandidateActive = false;
    turnState = TURN_NONE;
    lineLocked = false;
    acquiring = missing = false;
    clearPidState();
}
} // namespace

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = (baseSpeed < 0) ? 0 : (baseSpeed > 255 ? 255 : baseSpeed);
    pidConfig.minSpeed = 0;
    autoMode = false;
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
    startBoxCheckPending = enabled; // Inspect the first post-START sensor sample.
    motor_stop(); // Motor command issued only from the next sampled PID frame.
}
bool pid_is_auto_mode(void) { return autoMode; }

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

    // All sixteen black is expected at the START, not an immediate finish.
    if (startBoxCheckPending) {
        startBoxCheckPending = false;
        if (bitCount16(mask) >= 14) { // tolerate up to two noisy eyes on the black box
            startBoxEscaping = true;
            startBoxStartedMs = now;
            startBoxLineFrames = 0;
            startBoxLineSinceMs = 0;
        }
    }
    if (startBoxEscaping) {
        if ((uint32_t)(now - startBoxStartedMs) >= START_BOX_TIMEOUT_MS) {
            startBoxEscaping = false;
            startBoxTimedOut = true;
            return LINE_CASE_STOPPED;
        }
        // Do not infer a position from a fully black box. Wait until
        // a narrow contiguous band appears (stable over three frames).
        const bool narrowLine =
            blackCount >= 2 && blackCount <= 8 && hasAdjacentLineSensors(mask);
        if (narrowLine) {
            if (startBoxLineFrames == 0) startBoxLineSinceMs = now;
            if (startBoxLineFrames < START_BOX_LINE_FRAMES) ++startBoxLineFrames;
            if (startBoxLineFrames >= START_BOX_LINE_FRAMES &&
                (uint32_t)(now - startBoxLineSinceMs) >= START_BOX_LINE_STABLE_MS) {
                startBoxEscaping = false;
                startBoxLineFrames = 0;
                lineLocked = previouslyLocked = true;
                acquiring = missing = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                return LINE_CASE_NORMAL_PID;
            }
        } else {
            startBoxLineFrames = 0;
            startBoxLineSinceMs = 0;
        }
        // Includes 16-black and temporary all-white while leaving the box.
        return LINE_CASE_STARTUP_COAST;
    }
    if (startBoxTimedOut) return LINE_CASE_STOPPED;

    if (recoveryTimeoutLatched) return LINE_CASE_LOST_LINE;
    if (recoverySessionActive &&
        (uint32_t)(now - recoveryStartedMs) >= RECOVERY_SESSION_TIMEOUT_MS) {
        latchRecoveryTimeout();
        return LINE_CASE_LOST_LINE;
    }

    // During recovery, a wide region is ambiguous and NOT a finish line.
    if (!recoverySessionActive && mask == 0xFFFFU) {
        fullLineCandidate = true;
        if (fullLineFrames < 3) ++fullLineFrames;
        if (fullLineFrames >= 3) {
            finishLineLatched = true;
            turnState = TURN_NONE;
            centeredTurnFrames = 0;
            return LINE_CASE_STOPPED;
        }
    } else {
        fullLineCandidate = false;
        fullLineFrames = 0;
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

    // Keep PID steering while a corner remains visible; pivot only after line loss.
    if (turnState != TURN_NONE) {
        if (isSearchTurn()) {
            const bool centeredLine = isCenteredNarrowLine(mask, position, blackCount);
            if (centeredLine) {
                if (!searchCandidateActive) {
                    searchCandidateActive = true;
                    searchCandidateSinceMs = now;
                }
                if ((uint32_t)(now - searchCandidateSinceMs) >=
                    SEARCH_REACQUIRE_CONFIRM_MS) {
                    turnState = TURN_NONE;
                    searchCandidateActive = false;
                    centeredTurnFrames = 0;
                    missing = false;
                    turnMissing = false;
                    settlingActive = true;
                    settleStartedMs = now;
                    settleInvalidSinceMs = 0;
                    derivativeHistoryValid = false;
                    derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                    lineLocked = true;
                    if (previouslyLocked) ++reacquisitions;
                    previouslyLocked = true;
                    return LINE_CASE_REACQUIRE_SETTLE;
                }
            } else {
                searchCandidateActive = false;
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
                lineLocked = true;
                if (previouslyLocked) ++reacquisitions;
                previouslyLocked = true;
                if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;
                return LINE_CASE_NORMAL_PID;
            }
        }

        if (!isSearchTurn() && blackCount == 0) {
            if (!turnMissing) {
                turnMissing = true;
                turnMissingSinceMs = now;
            } else if (uint32_t(now - turnMissingSinceMs) >= LINE_LOSS_MS) {
                beginRecoverySearch(turnState, now);
                clearPidState();
                return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
            }
        } else {
            turnMissing = false;
        }

        if (isSearchTurn() &&
            uint32_t(now - searchStartedMs) >= LINE_SEARCH_TIMEOUT_MS) {
            latchRecoveryTimeout();
            return LINE_CASE_LOST_LINE;
        }
        if (!isSearchTurn() && blackCount > 0)
            return LINE_CASE_NORMAL_PID;
        return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
    }

    if (settlingActive) {
        if (isCenteredNarrowLine(mask, position, blackCount)) {
            if (settleInvalidSinceMs != 0) {
                // Require a new full stable interval after a dropout.
                settleStartedMs = now;
            }
            settleInvalidSinceMs = 0;
            missing = false;
            if ((uint32_t)(now - settleStartedMs) >= REACQUIRE_SETTLE_MS) {
                settlingActive = false;
                recoverySessionActive = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                return LINE_CASE_NORMAL_PID;
            }
            return LINE_CASE_REACQUIRE_SETTLE;
        }
        // Do not drive through an ambiguous junction during recovery.
        if (settleInvalidSinceMs == 0) settleInvalidSinceMs = now;
        if ((uint32_t)(now - settleInvalidSinceMs) < REACQUIRE_INVALID_MS)
            return LINE_CASE_REACQUIRE_SETTLE;
        const TurnState direction = position < 0.0f ? TURN_LEFT :
            position > 0.0f ? TURN_RIGHT : lastTurnDirection;
        beginRecoverySearch(direction, now);
        clearPidState();
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
    beginRecoverySearch(lastKnownPosition < 0.0f ? TURN_LEFT :
                        lastKnownPosition > 0.0f ? TURN_RIGHT :
                        lastTurnDirection, now);
    clearPidState();
    return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
}

void pid_line_follow_step(void) {
    if (!autoMode) return;
    float pos = 0;
    uint8_t blackCount = 0;
    const bool good = pid_calculate_position(&pos, &blackCount);
    const LineCase_t prev = currentCase;
    currentCase = pid_detect_case(pos, blackCount);
    const uint32_t nowUs = micros();
    const float measuredDt = (float)(nowUs - lastComputeTimeUs) * 1e-6f;
    const bool dtValid = measuredDt > 0 && measuredDt <= 0.1f;
    const float dt = dtValid ? measuredDt : 0.01f;
    lastComputeTimeUs = nowUs;
    if (startBoxTimedOut) {
        motor_stop();
        autoMode = false; // A new GPIO34 press can explicitly attempt START again.
        return;
    }
    if (prev != LINE_CASE_NORMAL_PID || currentCase != LINE_CASE_NORMAL_PID)
        derivativeHistoryValid = false;

    if (currentCase == LINE_CASE_SHARP_LEFT) {
        clearPidState();
        if (turnState == SEARCH_LEFT) {
            if (searchCandidateActive) driveSearchCandidate(pos);
            else motor_set_speeds(-LINE_SEARCH_PWM, LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(inner, outer);
        }
    } else if (currentCase == LINE_CASE_SHARP_RIGHT) {
        clearPidState();
        if (turnState == SEARCH_RIGHT) {
            if (searchCandidateActive) driveSearchCandidate(pos);
            else motor_set_speeds(LINE_SEARCH_PWM, -LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(outer, inner);
        }
    } else if ((currentCase == LINE_CASE_NORMAL_PID ||
                currentCase == LINE_CASE_REACQUIRE_SETTLE) && good &&
               (currentCase != LINE_CASE_REACQUIRE_SETTLE ||
                isCenteredNarrowLine(sensor_get_line_bits(), pos, blackCount))) {
        const float err = pos;
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
        }
        lastError = err;
        derivativeHistoryValid = true;
        lastPTerm = p;
        lastITerm = it;
        lastDTerm = d;
        lastCorrection = p + it + d;

        // Only NORMAL_PID gets a higher nominal forward PWM.
        // Keep the original baseSpeed for start-box, search, sharp-turn and cross-line.
        const float normalPidBase = fminf(
            (float)pidConfig.maxSpeed,
            (float)pidConfig.baseSpeed * NORMAL_PID_BASE_GAIN);
        float base = updateForwardSpeed(absErr, normalPidBase);
        const uint16_t sensorMask = sensor_get_line_bits();
        const float edgeSeverity = outerSensorSeverity(sensorMask);
        if (edgeSeverity > 0.0f) {
            const float minimumSpeed = fminf(normalPidBase, 45.0f);
            const float edgeSpeed = fmaxf(
                minimumSpeed,
                normalPidBase *
                    (1.0f - OUTER_SENSOR_SPEED_REDUCTION * edgeSeverity));
            if (edgeSpeed < base) base = edgeSpeed;
            governedForwardSpeed = base;
        }
        const bool settling = currentCase == LINE_CASE_REACQUIRE_SETTLE;
        if (settling) {
            const float progress = fminf(
                (float)(millis() - settleStartedMs) / (float)REACQUIRE_SETTLE_MS,
                1.0f);
            const float rampSpeed = (float)DROPOUT_FORWARD_PWM +
                (base - (float)DROPOUT_FORWARD_PWM) * progress;
            if (rampSpeed < base) base = rampSpeed;
        }
        // Normal_PID remains at 2B, while settling forbids reversing a wheel.
        const float correctionLimit = base *
            (settling ? REACQUIRE_CORRECTION_MULTIPLIER :
                        PID_CORRECTION_BASE_MULTIPLIER);
        float correction = lastCorrection;
        if (correction > correctionLimit) correction = correctionLimit;
        if (correction < -correctionLimit) correction = -correctionLimit;

        // Save correction for dashed-line coast
        lastDashedCorrection = correction;

        float leftCommand = base + correction;
        float rightCommand = base - correction;
        const float outerErrorRatio = fminf(
            fmaxf((absErr - OUTER_STEERING_START_ERROR) /
                  (OUTER_STEERING_FULL_ERROR - OUTER_STEERING_START_ERROR),
                  0.0f),
            1.0f);
        const float steeringSeverity = fmaxf(outerErrorRatio, edgeSeverity);
        const float steeringPeakLimit = base +
            ((float)pidConfig.maxSpeed - base) * steeringSeverity;
        limitMotorCommandPeak(leftCommand, rightCommand, steeringPeakLimit);
        motor_set_speeds(clampPwmSigned(leftCommand), clampPwmSigned(rightCommand));
    } else if (currentCase == LINE_CASE_CROSS_LINE) {
        clearPidState();
        const int16_t speed = clampPwm((float)pidConfig.baseSpeed * 0.7f);
        motor_set_speeds(speed, speed);
    } else if (currentCase == LINE_CASE_DASHED_LINE) {
        // Instead of going straight, coast with the last known correction.
        // This maintains the turn trajectory through brief line gaps.
        const float coastBase = fminf(governedForwardSpeed,
                                      (float)DROPOUT_FORWARD_PWM);
        const float coastCorrection = lastDashedCorrection * 0.7f; // damped
        float leftCommand = coastBase + coastCorrection;
        float rightCommand = coastBase - coastCorrection;
        limitMotorCommandPeak(leftCommand, rightCommand, coastBase);
        motor_set_speeds(clampPwmSigned(leftCommand),
                         clampPwmSigned(rightCommand));
    } else if (currentCase == LINE_CASE_STARTUP_COAST) {
        // The start box requires enough PWM to overcome static friction, even
        // when the tuned NORMAL_PID Base is deliberately conservative.
        const int16_t speed = startBoxEscaping ? START_BOX_PWM :
            (pidConfig.baseSpeed < DROPOUT_FORWARD_PWM ?
             pidConfig.baseSpeed : DROPOUT_FORWARD_PWM);
        motor_set_speeds(speed, speed);
    } else {
        clearPidState();
        motor_stop(); // Safe stop after confirmed line loss or any sensor fault.
    }
}

LineCase_t pid_get_current_case(void) { return currentCase; }
bool pid_line_is_confirmed(void) { return lineLocked; }
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
        case LINE_CASE_REACQUIRE_SETTLE: return "CASE 11: Reacquire - slow stable tracking";
        default: return "Khong Xac Dinh";
    }
}
float pid_get_last_error(void) { return lastError; }
float pid_get_correction(void) { return lastCorrection; }
float pid_get_term_p(void) { return (currentCase == LINE_CASE_NORMAL_PID || currentCase == LINE_CASE_REACQUIRE_SETTLE) ? lastPTerm : 0.0f; }
float pid_get_term_i(void) { return (currentCase == LINE_CASE_NORMAL_PID || currentCase == LINE_CASE_REACQUIRE_SETTLE) ? lastITerm : 0.0f; }
float pid_get_term_d(void) { return (currentCase == LINE_CASE_NORMAL_PID || currentCase == LINE_CASE_REACQUIRE_SETTLE) ? lastDTerm : 0.0f; }
float pid_get_kp(void) { return pidConfig.Kp; }
float pid_get_ki(void) { return pidConfig.Ki; }
float pid_get_kd(void) { return pidConfig.Kd; }
int16_t pid_get_base_speed(void) { return pidConfig.baseSpeed; }
