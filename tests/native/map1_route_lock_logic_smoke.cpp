// Pure C++ FSM regression:
// g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/native/map1_route_lock_logic_smoke.cpp -o /tmp/map1_route_lock_logic_smoke
// /tmp/map1_route_lock_logic_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "map1_route_lock.h"

int main() {
    map1route::Tracker t;
    t.begin(true);
    assert(t.stage == map1route::STRAIGHT_1);
    assert(t.baseCap(100, 140) == 140);
    assert(t.baseCap(230, 140) == 130);
    t.observe(260, 0.0f, true, true, 10);
    assert(t.stage == map1route::SEMICIRCLE);
    assert(t.firstRightSearchWindow(640));
    assert(t.baseCap(600, 140) == 100.0f);
    assert(t.wheelCap(600) == 170.0f);
    t.observe(1050, -48.0f, true, true, 400);
    assert(t.sawLeftApex);
    t.observe(1370, 27.0f, true, true, 800);
    assert(t.stage == map1route::RIGHT_TURN);
    assert(t.baseCap(1450, 140) == 110.0f);
    t.observe(1640, 2.0f, true, true, 900);
    t.observe(1650, 1.0f, true, true, 945);
    assert(t.stage == map1route::STRAIGHT_2);
    assert(t.baseCap(16500, 220) == 220.0f);
    uint16_t cal[16] = {};
    for (int i=0;i<16;i++) cal[i]=1000;
    float pos=0.0f; uint8_t count=0;
    t.begin(true);
    t.observe(260, 0, true, true, 100);
    // Two separate line clusters at S2,S3 and S10,S11.
    uint16_t pattern = (uint16_t)((1U<<2)|(1U<<3)|(1U<<10)|(1U<<11));
    assert(t.chooseCluster(pattern, cal, -35.0f, 1000, pos, count));
    assert(pos == -50.0f && count == 2);
    assert(t.selectedMask == ((1U<<2)|(1U<<3)));
    t.observe(1050, -40, true, true, 500);
    t.observe(1360, 30, true, true, 530);
    assert(t.stage == map1route::RIGHT_TURN);
    assert(t.chooseCluster(pattern, cal, -35.0f, 1400, pos, count));
    assert(pos == 30.0f && count == 2);
    assert(t.selectedMask == ((1U<<10)|(1U<<11)));
    // Wrong sequence should not silently advance to straight.
    t.begin(true);
    t.observe(260, 0, true, true, 100);
    t.observe(1700, 0, true, true, 200);
    assert(t.stage == map1route::FAULT);
    assert(t.fault == map1route::MISSING_ARC_CUE);
    // Missing stable exit causes a separate route fault.
    t.begin(true);t.observe(260,0,true,true,100);
    t.observe(1020,-40,true,true,200);t.observe(1360,25,true,true,300);
    t.observe(1930,25,true,true,400);
    assert(t.stage == map1route::FAULT);
    assert(t.fault == map1route::MISSING_RIGHT_EXIT);
    // SEARCH timeout survives encoder drift beyond the 540..800 window.
    t.begin(true);
    t.observe(260, 0, true, true, 100);
    t.monitorFirstSearch(650, true, true, 200);
    assert(t.firstSearchActive);
    t.monitorFirstSearch(900, true, false, 460);
    assert(t.stage == map1route::FAULT);
    assert(t.fault == map1route::FIRST_SEARCH_TIMEOUT);
    // Disabled mode does not change map 2/3 PID behavior.
    t.begin(false);
    t.observe(2000, 60, false, false, 500);
    assert(t.stage == map1route::OFF && t.baseCap(600,140)==140);
    puts("MAP1_ROUTE_LOCK_LOGIC_OK");
    return 0;
}
