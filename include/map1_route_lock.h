#ifndef MAP1_ROUTE_LOCK_H
#define MAP1_ROUTE_LOCK_H

// Map 01 FIRST 2k ticks only. Pure C++ route observer, no Arduino dependencies.
// Assumption: every run starts from the same physical pose/orientation.
// Encoder is a WINDOW cue, never an unconditional request to spin.
#include <stdint.h>
#include <math.h>

namespace map1route {

enum Stage : uint8_t {
    OFF=0, STRAIGHT_1=1, SEMICIRCLE=2, RIGHT_TURN=3,
    STRAIGHT_2=4, FAULT=5
};
enum FaultCode : uint8_t {
    NONE=0, FIRST_SEARCH_TIMEOUT=1, MISSING_ARC_CUE=2,
    MISSING_RIGHT_EXIT=3
};
struct Tracker {
    bool active = false;
    Stage stage = OFF;
    FaultCode fault = NONE;
    bool sawLeftApex = false;
    uint32_t centeredSinceMs = 0;
    uint16_t selectedMask = 0;
    bool firstSearchActive = false;
    uint32_t firstSearchStartedMs = 0;

    void begin(bool enable) {
        active = enable;
        stage = enable ? STRAIGHT_1 : OFF;
        fault = NONE;
        sawLeftApex = false;
        centeredSinceMs = 0;
        selectedMask = 0;
        firstSearchActive = false;
        firstSearchStartedMs = 0;
    }
    void fail(FaultCode code) {
        fault = code;
        stage = FAULT;
    }
    // validLine: FINISH armed, confirmed line, 1..10 black eyes.
    // normalPid is required only when releasing the right turn.
    void observe(int32_t ticks, float position, bool validLine,
                 bool normalPid, uint32_t nowMs) {
        if (!active || stage == FAULT || stage == OFF) return;
        switch (stage) {
        case STRAIGHT_1:
            if (ticks >= 250 && validLine) stage = SEMICIRCLE;
            else if (ticks > 550) fail(MISSING_ARC_CUE);
            break;
        case SEMICIRCLE:
            if (validLine && ticks >= 900 && ticks <= 1280 &&
                position <= -30.0f)
                sawLeftApex = true;
            if (sawLeftApex && validLine && ticks >= 1220 &&
                position >= 15.0f)
                stage = RIGHT_TURN;
            else if (ticks > 1680)
                fail(MISSING_ARC_CUE);
            break;
        case RIGHT_TURN:
            if (ticks >= 1540 && validLine && normalPid &&
                fabsf(position) <= 12.0f) {
                if (centeredSinceMs == 0U) centeredSinceMs = nowMs;
                else if ((uint32_t)(nowMs - centeredSinceMs) >= 40U)
                    stage = STRAIGHT_2;
            } else {
                centeredSinceMs = 0U;
            }
            if (ticks > 1920 && stage != STRAIGHT_2)
                fail(MISSING_RIGHT_EXIT);
            break;
        default: break;
        }
    }
    // Keep the 250ms bound even if wheel encoder drift moves beyond 800.
    // Do not reset the timer for a single noisy eye while SEARCH is active.
    void monitorFirstSearch(int32_t ticks, bool isSearching,
                            bool physicalWhite, uint32_t nowMs) {
        if (!active || stage != SEMICIRCLE) {
            firstSearchActive = false;
            return;
        }
        if (firstSearchActive) {
            if (!isSearching) firstSearchActive = false;
            else if ((uint32_t)(nowMs-firstSearchStartedMs) >= 250U)
                fail(FIRST_SEARCH_TIMEOUT);
        } else if (isSearching && physicalWhite &&
                   ticks >= 540 && ticks <= 800) {
            firstSearchActive = true;
            firstSearchStartedMs = nowMs;
        }
    }
    bool firstRightSearchWindow(int32_t ticks) const {
        return active && stage == SEMICIRCLE &&
               (firstSearchActive || (ticks >= 540 && ticks <= 800));
    }
    float baseCap(int32_t ticks, float normalBase) const {
        if (!active) return normalBase;
        if (stage == STRAIGHT_1)
            return ticks >= 200 && normalBase > 130 ? 130.0f : normalBase;
        if (stage == SEMICIRCLE) {
            const float cap = ticks >= 500 && ticks < 800 ? 100.0f :
                              ticks >= 800 ? 110.0f : 115.0f;
            return normalBase > cap ? cap : normalBase;
        }
        if (stage == RIGHT_TURN)
            return normalBase > 110 ? 110.0f : normalBase;
        return normalBase;
    }
    float wheelCap(int32_t ticks) const {
        if (!active) return 255.0f;
        if (stage == SEMICIRCLE)
            return ticks >= 500 && ticks < 800 ? 170.0f : 185.0f;
        if (stage == RIGHT_TURN) return 180.0f;
        return 255.0f;
    }

    // Return true ONLY if at least two separate valid black-eye clusters exist.
    // Select one complete contiguous group instead of averaging branches.
    // weights: -75,-65,...,+75; sample values are calibrated 0..1000.
    bool chooseCluster(uint16_t rawMask, const uint16_t samples[16],
                       float lastPos, int32_t ticks,
                       float& outPosition, uint8_t& outCount) {
        selectedMask = rawMask;
        if (!active || stage == STRAIGHT_2 || stage == FAULT) return false;
        uint8_t groups = 0, bestCount = 0;
        uint16_t bestMask = 0;
        float bestPos = 0.0f, bestScore = 1.0e9f;
        for (int i=0; i<16;) {
            if (!(rawMask & (uint16_t)(1U << i))) { ++i; continue; }
            float weighted = 0.0f;
            uint32_t total = 0;
            uint16_t groupMask = 0;
            uint8_t count = 0;
            while (i<16 && (rawMask & (uint16_t)(1U << i))) {
                const uint16_t sample = samples[i];
                weighted += (float)sample * ((float)i * 10.0f - 75.0f);
                total += sample;
                groupMask |= (uint16_t)(1U << i);
                ++i; ++count;
            }
            if (!total) continue;
            ++groups;
            const float center = weighted / (float)total;
            float score = fabsf(center-lastPos);
            // First part of semicircle should stay with the left-side line.
            if (stage == SEMICIRCLE && ticks >= 850 && ticks < 1280 &&
                center > 0.0f) score += 30.0f;
            // After the arc, prefer the commanded right branch.
            if (stage == RIGHT_TURN && center < 0.0f) score += 60.0f;
            if (score < bestScore) {
                bestScore=score;bestPos=center;
                bestCount=count;bestMask=groupMask;
            }
        }
        if (groups < 2 || bestMask == 0) return false;
        selectedMask=bestMask;
        outPosition=bestPos;
        outCount=bestCount;
        return true;
    }
};
} // namespace map1route
#endif
