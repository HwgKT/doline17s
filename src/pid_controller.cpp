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
constexpr int16_t DROPOUT_FORWARD_PWM = 65;
constexpr int16_t LINE_SEARCH_PWM = 85;
// Search reacquisition must be a continuously centered, narrow line in real time.
constexpr uint32_t SEARCH_REACQUIRE_CONFIRM_MS = 40;
constexpr uint32_t REACQUIRE_SETTLE_MS = 150;
constexpr uint32_t REACQUIRE_INVALID_MS = 40;
// Watchdog spans repeated SEARCH -> false reacquire -> SEARCH cycles.
constexpr uint32_t RECOVERY_SESSION_TIMEOUT_MS = 2500;
constexpr uint8_t REACQUIRE_DERIVATIVE_HOLD_FRAMES = 4;
constexpr float REACQUIRE_CORRECTION_MULTIPLIER = 1.0f;
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

// Ported from ECcodedoline with a sharper-corner safety envelope.
constexpr float EC_CORNER_DRAG_PER_ERROR = 0.55f;
constexpr float EC_HARD_CORNER_START_ERROR = 10.0f;
constexpr float EC_HARD_CORNER_FULL_ERROR = 35.0f;
constexpr float SPEED_GOVERNOR_REDUCTION = 0.45f;
constexpr float EC_STRAIGHT_CENTER_ERROR = 6.0f;
constexpr uint32_t EC_STRAIGHT_STABLE_MS = 140;
constexpr uint32_t EC_STRAIGHT_BOOST_RAMP_MS = 120;
constexpr float EC_STRAIGHT_BOOST_PWM = 20.0f;
constexpr float EC_STRAIGHT_BOOST_LIMIT = 215.0f;
constexpr float EC_P_GAIN_ERROR_SLOPE = 0.012f;
constexpr float EC_D_FILTER_ALPHA = 0.20f;
constexpr float EC_D_MAX_BASE_FRACTION = 0.55f;
constexpr float EC_D_MAX_ABSOLUTE = 120.0f;
// ECcodedoline 3-phase sharp corner (time-based, never delay/block SPP).
constexpr uint32_t EC_SHARP_BRAKE_MS = 22;
constexpr uint32_t EC_SHARP_COUNTER_BRAKE_MS = 18;
constexpr int16_t EC_SHARP_BRAKE_REVERSE_PWM = 110;
constexpr int16_t EC_SHARP_BRAKE_FORWARD_PWM = 35;
constexpr int16_t EC_SHARP_PIVOT_INNER_PWM = 105;
constexpr int16_t EC_SHARP_PIVOT_OUTER_PWM = 115;
constexpr int16_t EC_SHARP_COUNTER_BRAKE_PWM = 45;
constexpr uint32_t EC_SHARP_REENTRY_COOLDOWN_MS = 80;
constexpr uint32_t EC_CROSS_COOLDOWN_MS = 120;
constexpr uint32_t EC_T_JUNCTION_CONFIRM_MS = 20;
constexpr uint32_t EC_T_JUNCTION_COOLDOWN_MS = 120;
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
bool fullLineCandidate = false;
bool finishLineLatched = false;
uint8_t fullLineFrames = 0;

bool lineLocked = false, previouslyLocked = false;
bool acquiring = false, missing = false;
uint32_t acquireSinceMs = 0, missingSinceMs = 0;
uint32_t confirmedLosses = 0, reacquisitions = 0;
float governedForwardSpeed = 0.0f;
uint32_t ecCenteredSinceMs = 0;
uint32_t ecLastCrossMs = 0;
uint32_t ecLastSharpExitMs = 0;
uint32_t ecCounterBrakeUntilMs = 0;
int8_t ecCounterBrakeSide = 0; // -1 left, +1 right
bool ecShapeQualified = false;
bool ecActiveSharpSearch = false;
int8_t ecTCandidate = 0;
uint32_t ecTCandidateSinceMs = 0;
uint32_t ecLastTJunctionExitMs = 0;

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
// A real narrow line must form ONE contiguous black run. An isolated second
// cluster elsewhere on the sensor array is junction/geometry ambiguity.
static bool isSingleContiguousRun(uint16_t mask) {
    bool sawBlack = false;
    bool sawGapAfterBlack = false;
    for (uint8_t i = 0; i < N_SENSORS; ++i) {
        if ((mask & (uint16_t)(1U << i)) != 0) {
            if (sawGapAfterBlack) return false;
            sawBlack = true;
        } else if (sawBlack) {
            sawGapAfterBlack = true;
        }
    }
    return sawBlack;
}
static bool isCenteredNarrowLine(uint16_t mask, float position, uint8_t blackCount) {
    return blackCount > 0 && blackCount <= 6 &&
           fabsf(position) <= TURN_CENTERED_POSITION &&
           (mask & TURN_CENTER_SENSOR_MASK) != 0 &&
           isSingleContiguousRun(mask);
}
static void limitMotorCommandPeak(float& left, float& right, float limit) {
    const float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > limit && peak > 0.0f) {
        const float scale = limit / peak;
        left *= scale;
        right *= scale;
    }
}
static float updateForwardSpeed(float absErr, uint16_t mask, bool settling) {
    const uint32_t now = millis();
    const uint8_t black = bitCount16(mask);
    const bool centered = !settling && absErr <= EC_STRAIGHT_CENTER_ERROR &&
        black > 0 && black <= 4 && (mask & TURN_CENTER_SENSOR_MASK) != 0 &&
        isSingleContiguousRun(mask) && turnState == TURN_NONE &&
        !recoverySessionActive;
    if (!centered) ecCenteredSinceMs = 0;
    else if (ecCenteredSinceMs == 0) ecCenteredSinceMs = now;

    float nominal = (float)pidConfig.baseSpeed;
    if (centered && (uint32_t)(now - ecCenteredSinceMs) >= EC_STRAIGHT_STABLE_MS) {
        const float elapsed = (float)((uint32_t)(now - ecCenteredSinceMs) -
            EC_STRAIGHT_STABLE_MS);
        const float progress = fminf(elapsed / EC_STRAIGHT_BOOST_RAMP_MS, 1.0f);
        const float ceiling = fminf((float)pidConfig.maxSpeed,
            fmaxf(nominal, EC_STRAIGHT_BOOST_LIMIT));
        nominal += (fminf(ceiling, nominal + EC_STRAIGHT_BOOST_PWM) -
                    nominal) * progress;
    }

    // ECcodedoline's proportional corner drag.
    const float sampleCornerSpeed = nominal - EC_CORNER_DRAG_PER_ERROR * absErr;
    // Protect high-error turns; do not use the sample's aggressive speed blindly.
    const float severeRatio = fminf(fmaxf(
        (absErr - EC_HARD_CORNER_START_ERROR) /
        (EC_HARD_CORNER_FULL_ERROR - EC_HARD_CORNER_START_ERROR), 0.0f), 1.0f);
    const float guardedCornerSpeed =
        nominal * (1.0f - SPEED_GOVERNOR_REDUCTION * severeRatio);
    float speed = fminf(sampleCornerSpeed, guardedCornerSpeed);
    const float minimum = fminf(nominal, 45.0f);
    if (speed < minimum) speed = minimum;
    if (speed > pidConfig.maxSpeed) speed = pidConfig.maxSpeed;
    governedForwardSpeed = speed;
    return speed;
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
    fullLineCandidate = false;
    finishLineLatched = false;
    fullLineFrames = 0;
    lastDashedCorrection = 0;
    governedForwardSpeed = 0.0f;
    ecCenteredSinceMs = 0;
    ecLastCrossMs = 0;
    ecLastSharpExitMs = 0;
    ecCounterBrakeUntilMs = 0;
    ecCounterBrakeSide = 0;
    ecShapeQualified = false;
    ecActiveSharpSearch = false;
    ecTCandidate = 0;
    ecTCandidateSinceMs = 0;
    ecLastTJunctionExitMs = 0;
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
    // Only an explicitly outer-edge-qualified corner gets EC's aggressive
    // three-phase pivot; generic lost-line recovery retains +/-85 PWM.
    ecActiveSharpSearch = (turnState == TURN_LEFT || turnState == TURN_RIGHT) &&
        ecShapeQualified;
    ecShapeQualified = false;
    ecCounterBrakeUntilMs = 0;
    if (!recoverySessionActive) {
        recoverySessionActive = true;
        recoveryStartedMs = now;
    }
    ++confirmedLosses;
    lineLocked = false; // SEARCH does not count as a confirmed line lock.
    lastTurnDirection = direction;
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
    motor_stop(); // Never start blind; wait for stable black-line detection.
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

    // A confirmed recovery failure stays stopped until AUTO is restarted.
    if (recoveryTimeoutLatched) return LINE_CASE_LOST_LINE;
    if (recoverySessionActive &&
        (uint32_t)(now - recoveryStartedMs) >= RECOVERY_SESSION_TIMEOUT_MS) {
        latchRecoveryTimeout();
        return LINE_CASE_LOST_LINE;
    }

    // Sample code: a 120ms cooldown keeps junction edges from rearming a turn.
    if (!recoverySessionActive && bitCount16(mask) >= 11)
        ecLastCrossMs = now;

    // During recovery, a wide region must NOT be mistaken for a finish line.
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
                    // Complete the EC pivot with a short counter-brake only
                    // for an actual qualified 90-degree edge, not normal search.
                    if (ecActiveSharpSearch) {
                        ecCounterBrakeSide = isLeftTurn() ? -1 : 1;
                        ecCounterBrakeUntilMs = now + EC_SHARP_COUNTER_BRAKE_MS;
                        ecLastSharpExitMs = now;
                    }
                    ecActiveSharpSearch = false;
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
                if (ecShapeQualified) ecLastSharpExitMs = now;
                ecShapeQualified = false;
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
            // Any intermittent dropout restarts the full continuous settle time.
            if (settleInvalidSinceMs != 0) settleStartedMs = now;
            settleInvalidSinceMs = 0;
            missing = false;
            if ((uint32_t)(now - settleStartedMs) >= REACQUIRE_SETTLE_MS) {
                settlingActive = false;
                recoverySessionActive = false; // Only sustained tracking clears watchdog.
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                return LINE_CASE_NORMAL_PID;
            }
            return LINE_CASE_REACQUIRE_SETTLE;
        }
        // Stop if the line is wide, ambiguous, or missing while settling.
        // Permit a short sensor dropout, but do not drive blindly.
        if (settleInvalidSinceMs == 0) {
            settleInvalidSinceMs = now;
            settleStartedMs = now; // Require a fresh continuous stable interval.
        }
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
        if (bitCount16(mask) >= 11) {
            ecTCandidate = 0;
            return LINE_CASE_CROSS_LINE;
        }

        const uint8_t leftOuter = bitCount16(mask & 0x000FU);
        const uint8_t rightOuter = bitCount16(mask & 0xF000U);
        const uint8_t leftHalf = bitCount16(mask & 0x00FFU);
        const uint8_t rightHalf = bitCount16(mask & 0xFF00U);
        const bool centerPair = bitCount16(mask & TURN_CENTER_SENSOR_MASK) >= 2;
        const bool cooldownClear =
            (uint32_t)(now - ecLastCrossMs) >= EC_CROSS_COOLDOWN_MS &&
            (uint32_t)(now - ecLastSharpExitMs) >= EC_SHARP_REENTRY_COOLDOWN_MS;
        // ECcodedoline T-junction signature: center + outer branch while
        // opposite outer sensors remain clear. 20ms deglitch before turning.
        const int8_t tCandidate = (centerPair && leftOuter >= 2 && rightOuter == 0 &&
            rightHalf <= 2) ? -1 :
            (centerPair && rightOuter >= 2 && leftOuter == 0 &&
            leftHalf <= 2) ? 1 : 0;
        if (ecTCandidate != tCandidate) {
            ecTCandidate = tCandidate;
            ecTCandidateSinceMs = now;
        }
        if (cooldownClear && tCandidate != 0 &&
            (uint32_t)(now - ecLastTJunctionExitMs) >= EC_T_JUNCTION_COOLDOWN_MS &&
            (uint32_t)(now - ecTCandidateSinceMs) >= EC_T_JUNCTION_CONFIRM_MS) {
            return tCandidate < 0 ? LINE_CASE_T_LEFT : LINE_CASE_T_RIGHT;
        }
        const bool sharpLeft =
            position <= -SHARP_TURN_POSITION ||
            (position <= -SHARP_TURN_POSITION_WITH_OUTER_EYES && leftOuter >= 2);
        const bool sharpRight =
            position >= SHARP_TURN_POSITION ||
            (position >= SHARP_TURN_POSITION_WITH_OUTER_EYES && rightOuter >= 2);

        if (sharpLeft || sharpRight) {
            const bool qualifiedEdge = cooldownClear &&
                ((sharpLeft && leftOuter >= 3 && rightHalf == 0) ||
                 (sharpRight && rightOuter >= 3 && leftHalf == 0));
            ecShapeQualified = qualifiedEdge;
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
    if (prev != LINE_CASE_NORMAL_PID || currentCase != LINE_CASE_NORMAL_PID) {
        derivativeHistoryValid = false;
        ecCenteredSinceMs = 0;
    }
    if ((prev == LINE_CASE_T_LEFT || prev == LINE_CASE_T_RIGHT) &&
        currentCase != prev) {
        ecLastTJunctionExitMs = millis();
        ecTCandidate = 0;
    }
    // Opposite pulse only after a verified centered reacquisition from an
    // EC-shaped corner. If center is lost, the pulse is cancelled immediately.
    if (ecCounterBrakeUntilMs != 0) {
        if (currentCase == LINE_CASE_REACQUIRE_SETTLE && good &&
            isCenteredNarrowLine(sensor_get_line_bits(), pos, blackCount) &&
            (int32_t)(ecCounterBrakeUntilMs - millis()) > 0) {
            const int16_t b = EC_SHARP_COUNTER_BRAKE_PWM;
            lastPTerm = lastITerm = lastDTerm = lastCorrection = 0.0f;
            motor_set_speeds(ecCounterBrakeSide < 0 ? b : -b,
                             ecCounterBrakeSide < 0 ? -b : b);
            return;
        }
        ecCounterBrakeUntilMs = 0;
    }

    if (currentCase == LINE_CASE_SHARP_LEFT) {
        clearPidState();
        if (turnState == SEARCH_LEFT) {
            if (searchCandidateActive) motor_stop(); // Validate centered line without pivoting.
            else if (ecActiveSharpSearch) {
                const uint32_t e = (uint32_t)(millis() - searchStartedMs);
                motor_set_speeds(
                    e < EC_SHARP_BRAKE_MS ? -EC_SHARP_BRAKE_REVERSE_PWM : -EC_SHARP_PIVOT_INNER_PWM,
                    e < EC_SHARP_BRAKE_MS ? EC_SHARP_BRAKE_FORWARD_PWM : EC_SHARP_PIVOT_OUTER_PWM);
            } else motor_set_speeds(-LINE_SEARCH_PWM, LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(inner, outer);
        }
    } else if (currentCase == LINE_CASE_SHARP_RIGHT) {
        clearPidState();
        if (turnState == SEARCH_RIGHT) {
            if (searchCandidateActive) motor_stop();
            else if (ecActiveSharpSearch) {
                const uint32_t e = (uint32_t)(millis() - searchStartedMs);
                motor_set_speeds(
                    e < EC_SHARP_BRAKE_MS ? EC_SHARP_BRAKE_FORWARD_PWM : EC_SHARP_PIVOT_OUTER_PWM,
                    e < EC_SHARP_BRAKE_MS ? -EC_SHARP_BRAKE_REVERSE_PWM : -EC_SHARP_PIVOT_INNER_PWM);
            } else motor_set_speeds(LINE_SEARCH_PWM, -LINE_SEARCH_PWM);
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
        const float p = pidConfig.Kp * (1.0f + EC_P_GAIN_ERROR_SLOPE * absErr) * err;

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
            // Stronger filtered damping; preserve existing time-based SPP KD units.
            d = EC_D_FILTER_ALPHA * rawD +
                (1.0f - EC_D_FILTER_ALPHA) * lastDTerm;
            const float maxD = fminf(EC_D_MAX_ABSOLUTE,
                (float)pidConfig.baseSpeed * EC_D_MAX_BASE_FRACTION);
            d = fmaxf(-maxD, fminf(d, maxD));
        }
        lastError = err;
        derivativeHistoryValid = true;
        lastPTerm = p;
        lastITerm = it;
        lastDTerm = d;
        lastCorrection = p + it + d;

        const uint16_t sensorMask = sensor_get_line_bits();
        const bool settling = currentCase == LINE_CASE_REACQUIRE_SETTLE;
        float base = updateForwardSpeed(absErr, sensorMask, settling);
        const float edgeSeverity = outerSensorSeverity(sensorMask);
        if (edgeSeverity > 0.0f) {
            const float minimumSpeed = (float)(pidConfig.baseSpeed < 45
                ? pidConfig.baseSpeed : 45);
            const float edgeSpeed = fmaxf(
                minimumSpeed,
                (float)pidConfig.baseSpeed *
                    (1.0f - OUTER_SENSOR_SPEED_REDUCTION * edgeSeverity));
            if (edgeSpeed < base) base = edgeSpeed;
            governedForwardSpeed = base;
        }
        if (settling) {
            const float elapsedRatio = fminf(
                (float)(millis() - settleStartedMs) / (float)REACQUIRE_SETTLE_MS,
                1.0f);
            const float rampSpeed = (float)DROPOUT_FORWARD_PWM +
                (base - (float)DROPOUT_FORWARD_PWM) * elapsedRatio;
            if (rampSpeed < base) base = rampSpeed;
        }
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
    } else if (currentCase == LINE_CASE_T_LEFT ||
               currentCase == LINE_CASE_T_RIGHT) {
        // EC sample T-branch steering, non-blocking and bounded by maxSpeed.
        clearPidState();
        const int16_t outer = clampPwm((float)pidConfig.baseSpeed);
        const int16_t inner = clampPwmSigned(-0.4f * (float)pidConfig.baseSpeed);
        if (currentCase == LINE_CASE_T_LEFT) motor_set_speeds(inner, outer);
        else motor_set_speeds(outer, inner);
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
