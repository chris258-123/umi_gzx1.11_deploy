# Migration Plan: Replace WSG Gripper with Franka Native Gripper

## Executive Summary

This plan outlines the steps to replace the WSG50 gripper with Franka's native gripper (Franka Hand) in the UMI system. The migration requires creating a new Franka gripper controller that matches the existing WSG controller interface while using Polymetis's GripperInterface through the zerorpc server.

## Analysis of Current Implementation

### Franka Gripper API Assessment (launch_franka_interface_server.py)

**Status**: ✅ The added Franka gripper API is **compatible** with UMI's deployment flow.

The user has correctly added the following methods to `FrankaInterface` class:

1. **`get_gripper_state()`** (lines 116-142):
   - Returns comprehensive state dict with timestamps
   - Includes: width, max_width, is_grasped, is_moving, command status
   - ✅ Provides all necessary information for observation collection

2. **`get_gripper_width()`** (lines 144-148):
   - Convenience method to get current width
   - ✅ Useful for quick state queries

3. **`goto_gripper_width()`** (lines 150-172):
   - Commands gripper to target width with speed/force control
   - ✅ Core control method for gripper actuation

4. **`grasp_gripper()`** (lines 174-191):
   - Force-controlled grasping with epsilon parameters
   - ✅ Advanced control for object manipulation

**Key Observations**:
- Thread-safe with `_gripper_lock` (line 17, 121, 162, 179)
- Proper error handling with try-except blocks
- Returns JSON-serializable dicts for zerorpc compatibility
- Timestamp conversion handles protobuf Timestamp objects
- Gracefully handles missing gripper (returns None/error dicts)

**Gap**: The server API uses **immediate commands** (`goto_gripper_width`), but UMI's control flow requires **trajectory-based scheduling** (`schedule_waypoint` with target timestamps). A controller layer is needed to bridge this gap.

## Architecture Overview

### Current WSG Architecture
```
BimanualUmiEnv → WSGController (multiprocess) → WSGBinaryDriver → WSG50 Hardware
                 ├─ schedule_waypoint()         (TCP, port 1000)
                 ├─ get_state()
                 └─ Shared memory ring buffer
```

### Target Franka Architecture
```
BimanualUmiEnv → FrankaGripperController (multiprocess) → FrankaInterface (zerorpc client) → launch_franka_interface_server.py → Polymetis GripperInterface → Franka Hand
                 ├─ schedule_waypoint()                                                      (zerorpc, port 4242)
                 ├─ get_state()
                 └─ Shared memory ring buffer
```

## Implementation Plan

### Phase 1: Create FrankaGripperController

**File**: `umi/real_world/franka_gripper_controller.py` (NEW)

**Requirements**:
1. Inherit from `multiprocessing.Process` (same as WSGController)
2. Use `SharedMemoryQueue` for command input
3. Use `SharedMemoryRingBuffer` for state output
4. Implement same public API as WSGController:
   - `schedule_waypoint(pos, target_time)`
   - `get_state(k=None, out=None)`
   - `get_all_state()`
   - `restart_put(start_time)`
   - `start()`, `stop()`, `is_ready` property

**Key Implementation Details**:

1. **Initialization Parameters**:
   ```python
   def __init__(self,
       shm_manager: SharedMemoryManager,
       robot_ip: str,           # NUC IP (same as robot)
       robot_port: int = 4242,  # zerorpc port
       frequency: int = 30,     # Control loop frequency
       move_max_speed: float = 0.2,  # m/s (Franka Hand max ~0.2 m/s)
       move_max_force: float = 70.0,  # N (Franka Hand max ~70N)
       receive_latency: float = 0.0,
       launch_timeout: float = 3,
       verbose: bool = False
   )
   ```

2. **Main Control Loop** (in `run()` method):
   - Connect to server via zerorpc: `zerorpc.Client()` → `tcp://{robot_ip}:{robot_port}`
   - Initialize trajectory interpolator: `PoseTrajectoryInterpolator` (1D, only gripper width)
   - Loop at specified frequency (30Hz recommended):
     - Interpolate target width at current time
     - Call `server.goto_gripper_width(width, speed, force, blocking=False)`
     - Call `server.get_gripper_state()` to retrieve state
     - Store state in ring buffer with latency-compensated timestamp
     - Process commands from input queue (SCHEDULE_WAYPOINT, RESTART_PUT, SHUTDOWN)
     - Regulate timing with `precise_wait()`

3. **State Format** (must match WSG for compatibility):
   ```python
   {
       'gripper_state': int,           # Status code
       'gripper_position': float,      # Width in meters
       'gripper_velocity': float,      # Velocity in m/s
       'gripper_force': float,         # Force in N
       'gripper_measure_timestamp': float,
       'gripper_receive_timestamp': float,
       'gripper_timestamp': float      # Latency-compensated
   }
   ```

4. **Trajectory Interpolation**:
   - Use `PoseTrajectoryInterpolator` with 1D pose: `[width, 0, 0, 0, 0, 0]`
   - Respect `move_max_speed` when scheduling waypoints
   - Compute velocity from trajectory derivative for smooth control

5. **Speed Calculation**:
   ```python
   dt = 1 / frequency
   target_width = pose_interp(t_now)[0]
   target_speed = abs((target_width - pose_interp(t_now - dt)[0]) / dt)
   target_speed = min(target_speed, move_max_speed)
   ```

### Phase 2: Update BimanualUmiEnv

**File**: `umi/real_world/bimanual_umi_env.py`

**Changes Required**:

1. **Import Statement** (after line 8):
   ```python
   from umi.real_world.franka_gripper_controller import FrankaGripperController
   ```

2. **Gripper Initialization** (lines 247-256, replace):
   ```python
   for gc, rc in zip(grippers_config, robots_config):
       # Determine gripper type from robot type or explicit config
       gripper_type = gc.get('gripper_type', 'wsg')  # Default to WSG for backward compatibility

       # Auto-detect: if robot is Franka, use Franka gripper
       if rc['robot_type'].startswith('franka') and gripper_type == 'wsg':
           gripper_type = 'franka'

       if gripper_type == 'franka':
           this_gripper = FrankaGripperController(
               shm_manager=shm_manager,
               robot_ip=rc['robot_ip'],  # Use robot IP (NUC IP)
               robot_port=4242,
               frequency=30,
               move_max_speed=gc.get('gripper_max_speed', 0.2),
               move_max_force=gc.get('gripper_max_force', 70.0),
               receive_latency=gc['gripper_obs_latency'],
               verbose=False
           )
       else:  # WSG gripper
           this_gripper = WSGController(
               shm_manager=shm_manager,
               hostname=gc['gripper_ip'],
               port=gc['gripper_port'],
               receive_latency=gc['gripper_obs_latency'],
               use_meters=True
           )

       grippers.append(this_gripper)
   ```

**Rationale**:
- Auto-detect gripper type from robot type (Franka robot → Franka gripper)
- Allow explicit override via `gripper_type` config parameter
- Maintain backward compatibility with WSG gripper
- Use robot IP for Franka gripper (same NUC server)

### Phase 3: Update Configuration Files

**File**: `example/eval_robots_config.yaml`

**Changes Required**:

1. **Update Robot Configuration** (lines 4-15):
   ```yaml
   "robots": [
     {
       "robot_type": "franka",
       "robot_ip": "172.16.13.130",  # NUC IP (where server runs), NOT robot IP!
       "robot_obs_latency": 0.0001,
       "robot_action_latency": 0.1,
       "tcp_offset": 0.235,
       "height_threshold": -0.024,
       "sphere_radius": 0.1,
       "sphere_center": [0, -0.06, -0.185]
     }
   ]
   ```

2. **Update Gripper Configuration** (lines 25-29):
   ```yaml
   "grippers": [
     {
       "gripper_type": "franka",  # NEW: Explicit gripper type
       # For Franka gripper, these fields are NOT used (will use robot_ip from robots config)
       # "gripper_ip": "172.16.13.130",  # Not needed for Franka
       # "gripper_port": 4242,            # Not needed for Franka
       "gripper_obs_latency": 0.01,
       "gripper_action_latency": 0.1,
       "gripper_max_speed": 0.2,      # NEW: Max gripper speed (m/s)
       "gripper_max_force": 70.0      # NEW: Max gripper force (N)
     }
   ]
   ```

2. **For WSG Gripper** (backward compatibility):
   ```yaml
   "grippers": [
     {
       "gripper_type": "wsg",  # Explicit WSG type
       "gripper_ip": "192.168.0.18",
       "gripper_port": 1000,
       "gripper_obs_latency": 0.01,
       "gripper_action_latency": 0.1
     }
   ]
   ```

**Note**: If `gripper_type` is omitted, system will auto-detect based on robot type.

### Phase 4: Update Server Initialization

**File**: `scripts_real/launch_franka_interface_server.py`

**Changes Required**:

1. **Add Command-Line Arguments** (before line 193):
   ```python
   import argparse

   if __name__ == '__main__':
       parser = argparse.ArgumentParser()
       parser.add_argument('--host', type=str, default='localhost',
                          help='Polymetis server host')
       parser.add_argument('--enable-gripper', action='store_true', default=True,
                          help='Enable gripper interface')
       parser.add_argument('--gripper-ip', type=str, default='127.0.0.1',
                          help='Gripper server IP')
       parser.add_argument('--gripper-port', type=int, default=50052,
                          help='Gripper server port')
       parser.add_argument('--bind-address', type=str, default='0.0.0.0',
                          help='zerorpc bind address')
       parser.add_argument('--bind-port', type=int, default=4242,
                          help='zerorpc bind port')
       args = parser.parse_args()

       interface = FrankaInterface(
           host=args.host,
           enable_gripper=args.enable_gripper,
           gripper_ip=args.gripper_ip,
           gripper_port=args.gripper_port
       )

       s = zerorpc.Server(interface)
       s.bind(f"tcp://{args.bind_address}:{args.bind_port}")
       print(f"[FrankaInterface] Server listening on tcp://{args.bind_address}:{args.bind_port}")
       s.run()
   ```

**Rationale**:
- Make server configurable via command-line
- Allow disabling gripper for testing
- Support custom gripper server configuration

### Phase 5: Update Documentation

**File**: `franka_instruction.md`

**Changes Required**:

Add section after line 27:

```markdown
## Gripper Configuration

### Using Franka Native Gripper (Franka Hand)

The system automatically uses Franka's native gripper when `robot_type: 'franka'` is set. The gripper is controlled through the same Polymetis server running on the NUC.

**Configuration**:
```yaml
grippers: [
  {
    gripper_type: "franka",  # Optional, auto-detected from robot type
    gripper_obs_latency: 0.01,
    gripper_action_latency: 0.1,
    gripper_max_speed: 0.2,   # Max speed in m/s (Franka Hand: 0.2 m/s)
    gripper_max_force: 70.0   # Max force in N (Franka Hand: 70 N)
  }
]
```

**No additional hardware required** - gripper control is integrated with robot control through Polymetis.

### Using WSG50 Gripper (Alternative)

If using an external WSG50 gripper with Franka robot:
```yaml
grippers: [
  {
    gripper_type: "wsg",
    gripper_ip: "192.168.0.18",
    gripper_port: 1000,
    gripper_obs_latency: 0.01,
    gripper_action_latency: 0.1
  }
]
```

Requires separate WSG50 gripper hardware and network connection.
```

## Critical Files to Modify

### New Files
1. **`umi/real_world/franka_gripper_controller.py`** - New Franka gripper controller (primary implementation)

### Modified Files
1. **`umi/real_world/bimanual_umi_env.py`** (lines 247-256)
   - Add import for FrankaGripperController
   - Add conditional gripper instantiation logic

2. **`scripts_real/launch_franka_interface_server.py`** (lines 193-195)
   - Add command-line argument parsing
   - Make server configurable

3. **`example/eval_robots_config.yaml`** (lines 25-29)
   - Add `gripper_type` field
   - Add Franka-specific gripper parameters

4. **`franka_instruction.md`** (after line 27)
   - Add gripper configuration documentation

### Unchanged Files (API compatibility maintained)
- `umi/real_world/wsg_controller.py` - No changes needed
- `umi/real_world/wsg_binary_driver.py` - No changes needed
- `umi/real_world/franka_interpolation_controller.py` - No changes needed
- `scripts_real/eval_real.py` - No changes needed (uses environment API)
- `scripts_real/demo_real_bimanual_robots.py` - No changes needed

## API Mapping: WSG → Franka

| WSG Controller API | Franka Controller API | Server Method | Notes |
|-------------------|----------------------|---------------|-------|
| `schedule_waypoint(pos, target_time)` | `schedule_waypoint(pos, target_time)` | `goto_gripper_width(width, speed, force)` | Controller interpolates trajectory |
| `get_state(k=None)` | `get_state(k=None)` | `get_gripper_state()` | Returns same state dict format |
| `get_all_state()` | `get_all_state()` | `get_gripper_state()` | Retrieves all buffered states |
| `restart_put(start_time)` | `restart_put(start_time)` | N/A | Resets timing for episode recording |
| `start()`, `stop()` | `start()`, `stop()` | N/A | Process lifecycle management |
| `is_ready` | `is_ready` | N/A | Readiness check |

## State Dictionary Mapping

| Field | WSG Source | Franka Source | Conversion |
|-------|-----------|---------------|------------|
| `gripper_state` | WSG status code | `is_moving` → 0/1 | Map boolean to int |
| `gripper_position` | WSG position (mm → m) | `width` (m) | Direct mapping |
| `gripper_velocity` | WSG velocity | Computed from trajectory | Derivative of interpolated position |
| `gripper_force` | WSG force_motor | Not available | Set to 0.0 or NaN |
| `gripper_measure_timestamp` | WSG hardware timestamp | `timestamp` from state | Convert protobuf timestamp |
| `gripper_receive_timestamp` | `time.time()` | `server_receive_timestamp` | Direct mapping |
| `gripper_timestamp` | Latency-compensated | `timestamp - receive_latency` | Apply latency compensation |

## Configuration Changes Summary

### Before (WSG Gripper)
```yaml
robots: [
  {robot_type: "franka", robot_ip: "172.16.13.130", ...}  # NUC IP
]
grippers: [
  {gripper_ip: "192.168.0.18", gripper_port: 1000, ...}  # WSG gripper IP
]
```

### After (Franka Gripper)
```yaml
robots: [
  {robot_type: "franka", robot_ip: "172.16.13.130", ...}  # NUC IP (not robot IP!)
]
grippers: [
  {gripper_type: "franka", gripper_max_speed: 0.2, gripper_max_force: 70.0, ...}
]
```

**Key Changes**:
- Add `gripper_type: "franka"` (or omit for auto-detection)
- Remove `gripper_ip` and `gripper_port` (uses robot_ip which is NUC IP)
- Add `gripper_max_speed` and `gripper_max_force` parameters

**IMPORTANT IP Configuration**:
- For Franka setup: `robot_ip` should be the **NUC IP** (172.16.13.130), NOT the robot IP (172.16.0.8)
- The NUC runs both the robot control server and gripper control server
- Both robot controller and gripper controller connect to the same NUC IP via zerorpc (port 4242)
- The actual Franka robot IP (172.16.0.8) is only used by Polymetis on the NUC, not by the client

## Deployment Flow Verification

### 1. Server Launch (NUC)
```bash
# On NUC with realtime kernel (IP: 172.16.13.130)
python scripts_real/launch_franka_interface_server.py --enable-gripper
```
**Verification**: Check for "Initial gripper state" printout

**Test gripper separately**:
```bash
# On desktop
python scripts_real/test_franka_gripper.py --nuc_ip 172.16.13.130
```
**Expected output**: Gripper opens, closes, opens again with state printouts

### 2. Manual Control Test
```bash
# On desktop
python scripts_real/control_franka.py
```
**Verification**:
- SpaceMouse button 0/1 should open/close gripper
- Check gripper responds smoothly

### 3. Policy Evaluation
```bash
python eval_real.py --robot_config=example/eval_robots_config.yaml -i checkpoint.ckpt -o output_dir
```
**Verification**:
- Environment initializes without errors
- Gripper observations appear in observation dict
- Policy can control gripper width
- No latency or synchronization issues

### 4. Demonstration Collection
```bash
python scripts_real/demo_real_bimanual_robots.py --robot_config=example/eval_robots_config.yaml -o demo_dir
```
**Verification**:
- Gripper teleoperation works smoothly
- Gripper data recorded in dataset
- Timestamps properly synchronized

## Testing Checklist

### Unit Tests
- [ ] FrankaGripperController initializes correctly
- [ ] zerorpc connection to server succeeds
- [ ] `schedule_waypoint()` adds waypoints to trajectory
- [ ] `get_state()` returns properly formatted dict
- [ ] Trajectory interpolation produces smooth motion
- [ ] Speed limiting works correctly
- [ ] Process lifecycle (start/stop) works

### Integration Tests
- [ ] BimanualUmiEnv creates Franka gripper for Franka robot
- [ ] BimanualUmiEnv creates WSG gripper for UR5 robot
- [ ] Gripper observations appear in environment observations
- [ ] Gripper actions execute correctly
- [ ] Latency compensation works
- [ ] Episode recording includes gripper data

### System Tests
- [ ] Manual control with SpaceMouse
- [ ] Policy evaluation with trained checkpoint
- [ ] Demonstration collection
- [ ] Multi-arm coordination (if bimanual)
- [ ] Collision avoidance still works
- [ ] Performance: control frequency maintained

## Potential Issues and Mitigations

### Issue 1: Gripper Force Not Available
**Problem**: Franka gripper state doesn't provide force feedback in the same way as WSG.
**Mitigation**:
- Set `gripper_force` to 0.0 or NaN in state dict
- If force is critical, investigate Polymetis API for force sensing
- Consider using `is_grasped` flag as proxy for force threshold

### Issue 2: Different Speed Characteristics
**Problem**: Franka Hand max speed (~0.2 m/s) differs from WSG50 (~0.2 m/s in practice, but configurable).
**Mitigation**:
- Make `move_max_speed` configurable in gripper config
- Tune speed parameter based on task requirements
- Test with existing policies to ensure compatibility

### Issue 3: Gripper Width Range Differences
**Problem**: Franka Hand width range (0-0.08m) may differ from WSG50 (0-0.11m).
**Mitigation**:
- Document width ranges in configuration
- Add validation/clamping in controller
- May need to retrain policies if width range is critical

### Issue 4: Latency Differences
**Problem**: Franka gripper through zerorpc may have different latency than WSG TCP.
**Mitigation**:
- Measure actual latency using calibration scripts
- Update `gripper_obs_latency` and `gripper_action_latency` in config
- Use same latency compensation mechanism as WSG

### Issue 5: Synchronization with Robot Control
**Problem**: Gripper and robot commands go through same server, potential for interference.
**Mitigation**:
- Keep gripper control frequency (30Hz) lower than robot (200Hz)
- Use non-blocking gripper commands
- Monitor for timing issues during testing

## Performance Considerations

### Control Frequency
- **Robot**: 200Hz (FrankaInterpolationController)
- **Gripper**: 30Hz (recommended, same as WSG)
- **Rationale**: Gripper doesn't need high-frequency control, reduces server load

### Latency Budget
- **Observation latency**: ~10ms (same as WSG)
- **Action latency**: ~100ms (same as WSG)
- **zerorpc overhead**: ~1-5ms (negligible)

### Memory Usage
- Shared memory ring buffer: ~10 seconds of data at 30Hz = 300 samples
- Minimal increase compared to WSG

## Rollback Plan

If Franka gripper integration fails:

1. **Immediate**: Set `gripper_type: "wsg"` in config to revert to WSG gripper
2. **Code**: No changes to WSG controller, can switch back instantly
3. **Hardware**: Keep WSG50 gripper available as backup
4. **Testing**: Test both gripper types in parallel during transition

## Success Criteria

The migration is successful when:

1. ✅ FrankaGripperController implements same API as WSGController
2. ✅ BimanualUmiEnv works with both WSG and Franka grippers
3. ✅ Manual control (control_franka.py) works smoothly
4. ✅ Policy evaluation produces same quality results as WSG
5. ✅ Demonstration collection works without issues
6. ✅ No performance degradation (control frequency maintained)
7. ✅ Documentation updated and clear
8. ✅ Configuration is simple and intuitive

## Next Steps

After plan approval:

1. Implement `FrankaGripperController` class
2. Update `BimanualUmiEnv` with conditional gripper creation
3. Update configuration files
4. Update server with command-line arguments
5. Test with manual control
6. Test with policy evaluation
7. Update documentation
8. Validate with demonstration collection

## Estimated Complexity

- **Implementation**: ~200-300 lines of new code (FrankaGripperController)
- **Modifications**: ~50 lines of changes to existing files
- **Testing**: ~2-4 hours of integration testing
- **Documentation**: ~30 minutes

**Total effort**: Medium complexity, straightforward implementation following existing patterns.
