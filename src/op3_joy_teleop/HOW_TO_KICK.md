# How to Kick with Gamepad

## The Problem
You're trying **Button 4 + L1** but it doesn't work!

## Why It Doesn't Work
**L1 is the deadman button** and kicking requires the deadman to be **RELEASED**!

From config line 54:
```yaml
require_deadman_released_for_kick: true
```

From code line 945:
```python
if self._require_deadman_released_for_kick and self._deadman_active:
    return  # Kick is blocked!
```

## Correct Kick Sequence

### Step 1: RELEASE L1 (Deadman)
**Important:** Let go of L1 first!

### Step 2: Long Press Y (Button 4)
Hold **Y button** for **0.5 seconds** to enter kick mode.

You should see kick mode activate for 2 seconds.

### Step 3: Press L2 or R2 Within 2 Seconds
- **L2 (Button 8)** = Left kick (page 104)
- **R2 (Button 9)** = Right kick (page 218)

## Button Mapping Summary

| Button | Function | Notes |
|--------|----------|-------|
| L1 (6) | Deadman | Must be HELD for walking |
| L1 (6) | - | Must be RELEASED for kicking |
| Y (4) | Kick Mode | Long press 0.5s to activate |
| L2 (8) | Left Kick | Only works in kick mode, deadman released |
| R2 (9) | Right Kick | Only works in kick mode, deadman released |

## Visual Guide

```
❌ WRONG:
Hold L1 + Hold Y + Press L2
(Kick blocked because deadman is active!)

✅ CORRECT:
1. Release L1 (stop walking)
2. Hold Y for 0.5s (enter kick mode)
3. Press L2 (left kick) or R2 (right kick)
```

## Timing

- **Kick mode window:** 2.0 seconds (after releasing Y, you have 2s to kick)
- **Kick cooldown:** 1.0 second (wait 1s between kicks)
- **Kick timeout:** 5.0 seconds (kick action times out after 5s)

## Debugging

### Check if kick mode is activating:
```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /robotis/enable_ctrl_module
# Should see "action_module" when kick triggered
```

### Check if kick page is being sent:
```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /robotis/action/page_num
# Should see 104 (left kick) or 218 (right kick)
```

### Check movement completion:
```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /robotis/movement_done
# Should see "action" when kick completes
```

### Monitor joy input:
```bash
ros2 topic echo /joy
# Check button states:
# - Button 6 (L1) should be 0 (released)
# - Button 4 (Y) should be 1 when pressed
# - Button 8 (L2) or 9 (R2) should be 1 when pressed
```

## Common Issues

### Issue 1: "I press Y but nothing happens"
**Solution:** Hold Y for **0.5 seconds**, not just tap it.

### Issue 2: "I press L2/R2 but no kick"
**Solution:**
1. Did you activate kick mode first? (long press Y)
2. Is L1 released? (deadman must be off)
3. Did you wait too long? (only 2 second window)

### Issue 3: "Robot stops moving when I release L1"
**Solution:** This is correct! Walking requires L1, kicking requires L1 released. You can't walk and kick simultaneously.

### Issue 4: "Kick happens but robot doesn't move"
**Possible causes:**
1. Wrong page numbers (check `left_kick_page: 104`, `right_kick_page: 218`)
2. Action module not properly loaded
3. Robot not in proper stance

## Alternative: Allow Kicking With Deadman

If you want to kick while holding deadman, edit the config:

```yaml
# In config/op3_joy_teleop.yaml
require_deadman_released_for_kick: false
```

Then rebuild:
```bash
colcon build --packages-select op3_joy_teleop
source install/setup.bash
```

**Warning:** This may cause conflicts if you're using L2/R2 for turning!

## Full Workflow Example

**To kick during a match:**

1. **Walk to ball** - Hold L1 + move left stick
2. **Stop** - Release L1 (robot stops walking)
3. **Enter kick mode** - Hold Y for 0.5s
4. **Kick** - Press L2 (left foot) or R2 (right foot)
5. **Resume walking** - Hold L1 again + move stick

## Summary

**You CANNOT hold L1 while kicking!**

Correct sequence:
```
Release L1 → Hold Y (0.5s) → Press L2/R2
```

NOT:
```
Hold L1 + Press Y + Press L2  ❌
```
