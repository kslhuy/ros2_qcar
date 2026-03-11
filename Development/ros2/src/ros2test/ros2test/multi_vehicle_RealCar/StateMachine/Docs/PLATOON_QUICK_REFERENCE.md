# Platoon Setup - Quick Reference

## TL;DR - The Flow

### ✅ CORRECT SEQUENCE

```
1. SETUP_PLATOON_FORMATION  ← Must come first!
   └─ Tells vehicle its role (leader/follower)
   └─ Sets setup_complete = True
   
2. START_PLATOON            ← Can only happen after setup
   └─ Checks: is setup_complete == True?
   └─ If YES: Transition to appropriate state
   └─ If NO: Reject with warning
```

### ❌ WRONG SEQUENCE (Now Protected)

```
START_PLATOON (without setup first)
   ↓
Check: setup_complete == False?
   ↓
⚠️ WARNING: "SETUP_PLATOON_FORMATION not received yet!"
COMMAND IGNORED ✅
```

---

## What Changed?

### In `PlatoonController` class:

```python
# NEW attributes:
self.setup_complete = False      # Flag: has setup been received?
self.formation_data = {}         # Store the formation config
self.my_position = None          # This vehicle's position (1=leader, 2+=follower)

# Updated methods:
def setup_from_global_formation(...):
    # ... setup code ...
    self.setup_complete = True  # ← SET THE FLAG!
    
def disable_platoon_mode():
    # ... cleanup code ...
    self.setup_complete = False  # ← RESET THE FLAG!
```

### In State handlers:

```python
# BEFORE START_PLATOON transition:
if not self.vehicle_logic.platoon_controller.setup_complete:
    logger.warning("Setup not complete!")
    return None  # Reject command
    
# Only proceed if setup_complete = True
```

---

## Command Details

### SETUP_PLATOON_FORMATION
```json
{
  "type": "setup_platoon_formation",
  "formation": {
    "0": 1,    // Vehicle 0 → Position 1 (LEADER)
    "1": 2,    // Vehicle 1 → Position 2 (FOLLOWER)
    "2": 3     // Vehicle 2 → Position 3 (FOLLOWER)
  },
  "leader_id": 0
}
```
**Result:**
- Vehicle 0: `is_leader=True, setup_complete=True`
- Vehicle 1: `is_leader=False, leader_car_id=0, setup_complete=True`
- Vehicle 2: `is_leader=False, leader_car_id=0, setup_complete=True`

### START_PLATOON
```json
{
  "type": "start_platoon",
  "leader_id": 0
}
```
**Requires:** `setup_complete=True` ✅

**Result:**
- Leaders: Transition to `FOLLOWING_PATH` (leader mode)
- Followers: Transition to `FOLLOWING_LEADER` (following mode)

---

## State Values

### setup_complete

| Value | Meaning | Can send START_PLATOON? |
|-------|---------|------------------------|
| `False` | Setup not received yet | ❌ NO (ignored) |
| `True` | Setup received, ready | ✅ YES (accepted) |

### is_leader (only set after setup)

| Value | Meaning | Role |
|-------|---------|------|
| `True` | This vehicle is leader | Controls followers |
| `False` | This vehicle is follower | Follows leader |

### my_position (only set after setup)

| Value | Meaning |
|-------|---------|
| `1` | Leader position |
| `2, 3, 4...` | Follower position |

---

## Example: Correct Usage in Ground Station

```python
# Ground Station / GUI

# Step 1: Setup formation
formation = {
    0: 1,  # Vehicle 0 = Leader
    1: 2,  # Vehicle 1 = Follower
    2: 3   # Vehicle 2 = Follower
}

for car_id in [0, 1, 2]:
    send_command(car_id, {
        'type': 'setup_platoon_formation',
        'formation': formation,
        'leader_id': 0
    })
    # Now: vehicle.platoon_controller.setup_complete = True ✅

# Step 2: Start platoon (only after setup!)
for car_id in [0, 1, 2]:
    send_command(car_id, {
        'type': 'start_platoon',
        'leader_id': 0
    })
    # Check passes: setup_complete = True
    # Transition: Vehicle 0 → FOLLOWING_PATH
    #             Vehicle 1 → FOLLOWING_LEADER
    #             Vehicle 2 → FOLLOWING_LEADER
```

---

## Monitoring/Debugging

### Check if setup is complete:
```python
if vehicle.platoon_controller.setup_complete:
    print("✅ Platoon setup complete, can start")
else:
    print("❌ Platoon setup not started yet")
```

### Check vehicle role:
```python
if vehicle.platoon_controller.is_leader:
    print(f"🚗 This vehicle is LEADER")
else:
    print(f"🚗 This vehicle is FOLLOWER (following {vehicle.platoon_controller.leader_car_id})")
```

### Check vehicle position:
```python
pos = vehicle.platoon_controller.my_position
print(f"Position in platoon: {pos} (1=leader, 2+=follower)")
```

---

## Files Changed

1. **`Controller/platoon_controller.py`**
   - `__init__()`: Added setup_complete, formation_data, my_position
   - `setup_from_global_formation()`: Sets setup_complete=True
   - `disable_platoon_mode()`: Resets setup_complete=False

2. **`StateMachine/following_path_state.py`**
   - `handle_event()`: Added setup_complete check for START_PLATOON

3. **`StateMachine/waiting_for_start_state.py`**
   - `handle_event()`: Added setup_complete check for START_PLATOON

---

## Safety Guarantees

✅ Cannot transition to FOLLOWING_LEADER without setup
✅ Cannot skip role assignment
✅ Cannot lose track of leader assignment
✅ Safe to send duplicate SETUP commands (overwrites)
✅ Safe to reset with DISABLE_PLATOON (clears flag)

---

## Troubleshooting

### Problem: START_PLATOON is ignored
**Cause:** `setup_complete = False`
**Solution:** Send SETUP_PLATOON_FORMATION first

### Problem: Vehicle doesn't know if it's leader
**Cause:** SETUP_PLATOON_FORMATION not received
**Solution:** Check command was sent and received

### Problem: Wrong position in formation
**Cause:** Formation config error in SETUP_PLATOON_FORMATION
**Solution:** Verify formation dict maps correct car_id → position

### Problem: Vehicle won't disable platoon
**Cause:** Normal
**Solution:** Send DISABLE_PLATOON command
**Result:** setup_complete resets to False

---

## Version Info

- **Implementation Date:** 2025-12-07
- **Status:** ✅ Complete and tested
- **Backward Compatible:** Yes (existing code still works, just with safety checks)
