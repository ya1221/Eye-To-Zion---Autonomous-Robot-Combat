#!/bin/bash
# Comprehensive test for ShootingNode — exercises all mode/rate scenarios.
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

PASS=0
FAIL=0

pass() { echo "  ✅ PASS: $1"; PASS=$((PASS+1)); }
fail() { echo "  ❌ FAIL: $1"; FAIL=$((FAIL+1)); }

# Helper: publish a mode message
pub_mode() {
    ros2 topic pub --once /shooting_mode std_msgs/msg/String "data: '$1'" > /dev/null 2>&1 || true
    sleep 1
}

# Helper: get fire_rate_hz
get_rate() {
    ros2 param get /shooting_node fire_rate_hz 2>&1
}

# Helper: set fire_rate_hz
set_rate() {
    ros2 param set /shooting_node fire_rate_hz "$1" 2>&1
}

# Helper: call fire_once service
fire_once() {
    ros2 service call /shooting_node/fire_once std_srvs/srv/Trigger 2>&1
}

echo ""
echo "============================================"
echo " ShootingNode Test Suite"
echo "============================================"

# ── Test 1: Default state is auto mode with fire_rate_hz=2.0 ─────────────
echo ""
echo "── Test 1: Default state ──"
RATE=$(get_rate)
echo "  fire_rate_hz = $RATE"
if echo "$RATE" | grep -q "2.0"; then
    pass "Default fire_rate_hz is 2.0"
else
    fail "Default fire_rate_hz is NOT 2.0 (got: $RATE)"
fi

# ── Test 2: Change fire_rate_hz in auto mode (should succeed) ────────────
echo ""
echo "── Test 2: Change fire_rate_hz in auto mode ──"
RESULT=$(set_rate 5.0)
echo "  Result: $RESULT"
if echo "$RESULT" | grep -qi "successful"; then
    pass "fire_rate_hz changed to 5.0 in auto mode"
else
    fail "Could not change fire_rate_hz in auto mode"
fi

RATE=$(get_rate)
echo "  Verify: fire_rate_hz = $RATE"
if echo "$RATE" | grep -q "5.0"; then
    pass "fire_rate_hz confirmed as 5.0"
else
    fail "fire_rate_hz not 5.0 (got: $RATE)"
fi

# ── Test 3: Switch to single mode → fire_rate_hz resets to 2.0 ──────────
echo ""
echo "── Test 3: Switch to single mode ──"
pub_mode "single"

RATE=$(get_rate)
echo "  fire_rate_hz = $RATE"
if echo "$RATE" | grep -q "2.0"; then
    pass "fire_rate_hz reset to 2.0 after switching to single"
else
    fail "fire_rate_hz NOT reset (got: $RATE)"
fi

# ── Test 4: Cannot change fire_rate_hz in single mode ────────────────────
echo ""
echo "── Test 4: Try changing fire_rate_hz in single mode ──"
RESULT=$(set_rate 10.0)
echo "  Result: $RESULT"
if echo "$RESULT" | grep -qi "not set\|failed\|error\|rejected"; then
    pass "fire_rate_hz change correctly rejected in single mode"
else
    fail "fire_rate_hz change was NOT rejected in single mode"
fi

# Verify it's still 2.0
RATE=$(get_rate)
echo "  Verify: fire_rate_hz = $RATE"
if echo "$RATE" | grep -q "2.0"; then
    pass "fire_rate_hz still 2.0 after rejected change"
else
    fail "fire_rate_hz changed despite rejection (got: $RATE)"
fi

# ── Test 5: fire_once works in single mode ───────────────────────────────
echo ""
echo "── Test 5: fire_once in single mode ──"
RESULT=$(fire_once)
echo "  Result: $RESULT"
if echo "$RESULT" | grep -qi "success: true\|queued"; then
    pass "fire_once succeeded in single mode"
else
    fail "fire_once did NOT succeed in single mode"
fi

# ── Test 6: Switch to auto, then fire_once should be rejected ────────────
echo ""
echo "── Test 6: fire_once rejected in auto mode ──"
pub_mode "auto"

RESULT=$(fire_once)
echo "  Result: $RESULT"
if echo "$RESULT" | grep -qi "success: false\|cannot\|not.*single"; then
    pass "fire_once correctly rejected in auto mode"
else
    fail "fire_once was NOT rejected in auto mode"
fi

# ── Test 7: single→auto keeps fire_rate_hz=2.0 ──────────────────────────
echo ""
echo "── Test 7: single→auto keeps fire_rate_hz=2.0 (rule 3/4) ──"
pub_mode "single"
pub_mode "auto"

RATE=$(get_rate)
echo "  fire_rate_hz = $RATE"
if echo "$RATE" | grep -q "2.0"; then
    pass "fire_rate_hz is 2.0 after single→auto"
else
    fail "fire_rate_hz not 2.0 (got: $RATE)"
fi

# ── Test 8: Full cycle: auto(set 8.0) → single(reset 2.0) ───────────────
echo ""
echo "── Test 8: Full cycle: auto(set 8.0) → single(reset 2.0) ──"
set_rate 8.0 > /dev/null 2>&1
RATE=$(get_rate)
echo "  fire_rate_hz after set in auto = $RATE"
if echo "$RATE" | grep -q "8.0"; then
    pass "fire_rate_hz set to 8.0 in auto"
else
    fail "fire_rate_hz not 8.0 (got: $RATE)"
fi

pub_mode "single"
RATE=$(get_rate)
echo "  fire_rate_hz after switch to single = $RATE"
if echo "$RATE" | grep -q "2.0"; then
    pass "fire_rate_hz reset to 2.0 after switching to single"
else
    fail "fire_rate_hz NOT reset (got: $RATE)"
fi

# ── Test 9: Invalid mode ignored ─────────────────────────────────────────
echo ""
echo "── Test 9: Invalid mode message ignored ──"
pub_mode "auto"
pub_mode "burst"
sleep 1

# Should still be auto (burst was rejected), so rate change should work
RESULT=$(set_rate 3.0)
echo "  Set rate to 3.0 result: $RESULT"
if echo "$RESULT" | grep -qi "successful"; then
    pass "Still in auto mode after invalid 'burst' mode (rate change worked)"
else
    fail "Mode may have changed to invalid value"
fi

# ── Test 10: Verify /shooting_cmd is being published ─────────────────────
echo ""
echo "── Test 10: /shooting_cmd publishing check ──"
MSG=$(timeout 3 ros2 topic echo /shooting_cmd --once 2>&1 || true)
echo "  /shooting_cmd msg: $(echo $MSG | head -c 200)"
if echo "$MSG" | grep -qi "data:"; then
    pass "/shooting_cmd is publishing"
else
    fail "/shooting_cmd is NOT publishing"
fi

# ── Summary ──────────────────────────────────────────────────────────────
echo ""
echo "============================================"
echo " Results: $PASS passed, $FAIL failed"
echo "============================================"
echo ""

# Clean up: restore default rate
set_rate 2.0 > /dev/null 2>&1 || true
pub_mode "auto"
