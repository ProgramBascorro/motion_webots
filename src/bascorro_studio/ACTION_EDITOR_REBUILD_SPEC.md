# Bascorro Studio Action Editor Rebuild Spec

## Goal

Rebuild the OP3 action editor inside `bascorro_studio` as a clean, maintainable web tool that stays compatible with the existing ROBOTIS OP3 runtime:

- `op3_manager` remains the controller/bootstrap entrypoint.
- `op3_action_module` remains the runtime that loads and plays action pages.
- `op3_action_editor/scripts/action_yaml.py` remains the compatibility boundary for the legacy binary action file unless we explicitly replace the binary format later.

The rebuild should prioritize action authoring and execution for OP3 first, not a generic robot studio.

## What Exists Today

### Current Bascorro Studio pieces

- Web UI: `src/bascorro_studio/web/src/App.jsx`
- Current action editor UI: `src/bascorro_studio/web/src/ActionEditor.jsx`
- Apply/export bridge node: `src/bascorro_studio/bascorro_studio/apply_node.py`
- Metrics/event agent: `src/bascorro_studio/bascorro_studio/studio_agent.py`
- Asset server for URDF + meshes: `src/bascorro_studio/bascorro_studio/asset_server.py`
- Launcher: `src/bascorro_studio/launch/bascorro_studio.launch.py`
- Dev launcher script: `scripts/bascorro_studio.sh`

### Existing OP3 runtime pieces

- Controller bootstrap: `src/ROBOTIS-OP3/op3_manager/src/op3_manager.cpp`
- Action runtime: `src/ROBOTIS-OP3/op3_action_module/src/action_module.cpp`
- Legacy terminal editor: `src/ROBOTIS-OP3-Tools/op3_action_editor/src/action_editor.cpp`
- Binary/YAML converter: `src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/action_yaml.py`
- Joint map source of truth: `src/ROBOTIS-OP3/op3_manager/config/OP3.robot`

## Important Clarification

`op3_manager` does not define the action pages. It initializes the robot/controller stack and loads motion modules.

Action pages are owned by `op3_action_module`, which:

- loads the binary action file on startup
- exposes runtime play control over ROS topics/services
- executes page numbers against the loaded motion data

So the rebuild should treat:

- `op3_manager` as the environment that makes action playback possible
- `op3_action_module` as the action runtime
- `bascorro_studio` as the authoring and orchestration UI

## Runtime Contract We Must Preserve

### ROS interfaces

Action playback is currently driven by:

- topic: `/robotis/action/page_num` (`std_msgs/Int32`)
- topic: `/robotis/action/start_action` (`op3_action_module_msgs/StartAction`)
- service: `/robotis/action/is_running` (`op3_action_module_msgs/IsRunning`)
- topic: `/robotis/enable_ctrl_module` (`std_msgs/String`)
- topic: `/robotis/present_joint_states` (`sensor_msgs/JointState`)

Useful related controls:

- `/robotis/base/ini_pose`
- `/robotis/sync_write_item`
- `/webots/joint_positions`
- `/robotis_op3/joint_states`

### Module behavior

Before running an action page, the robot usually needs the control module switched to `action_module` through `/robotis/enable_ctrl_module`.

### Action file compatibility

The current compatible action file format is the legacy OP3 binary motion file handled by `action_yaml.py`.

Current format constraints from `action_yaml.py`:

- 256 pages total
- 512 bytes per page
- 7 steps max per page
- 31 joint slots per step in file format
- page header includes:
  - `name`
  - `repeat`
  - `schedule`
  - `stepnum`
  - `speed`
  - `accel`
  - `next`
  - `exit`
  - `pgain`
- schedule values:
  - `speed`
  - `time`
- step fields:
  - positions
  - `pause`
  - `time`
- position special values:
  - invalid / unset
  - torque off

The OP3 robot file currently defines 20 active joints:

- arms: shoulder pitch/roll, elbow
- legs: hip yaw/roll/pitch, knee, ankle pitch/roll
- head: pan, tilt

## Current ActionEditor Scope

The existing `ActionEditor.jsx` already includes more than a basic editor:

- YAML document editing
- page and step editing
- URDF preview with Three.js
- joint mirroring
- timeline preview
- undo/redo
- local draft persistence
- snapshots
- scratch run to a configurable temporary page
- export/apply through `/bascorro_studio/request`
- live joint pose from hardware or simulation

This is useful, but the file is too large and mixes:

- domain rules
- YAML document transforms
- ROS transport
- 3D viewer logic
- page editor UI
- timeline logic
- persistence/history

That is the main maintainability problem.

## Rebuild Target

### Product definition

The rebuilt action editor should be the canonical OP3 motion authoring tool for:

- creating action pages
- editing per-step joint targets
- previewing motions safely
- scratch-running motions in simulation or on hardware
- importing/exporting compatible YAML
- applying YAML back into the binary action file used by `op3_action_module`

### Non-goals for v1

Do not expand v1 into:

- full walking parameter tuning redesign
- full match dashboard redesign
- generalized multi-robot support
- replacing the OP3 action binary format
- live servo parameter calibration beyond the existing action-page scope

## Recommended Architecture

Split the current monolith into these layers.

### 1. Domain model

Create pure OP3 action editor utilities with no React and no ROS dependency.

Suggested responsibility:

- page normalization
- step normalization
- joint mapping
- raw <-> degrees/radians conversion
- mirror transforms
- schedule/time math
- YAML parse/serialize
- validation
- scratch page generation

Suggested folder:

- `src/bascorro_studio/web/src/action_editor/core/`

Suggested modules:

- `model.js`
- `yaml.js`
- `kinematics.js`
- `timeline.js`
- `validation.js`
- `scratch.js`

### 2. ROS gateway

Create a thin transport layer for ROS topics/services used by the editor.

Suggested responsibility:

- connect/disconnect state
- publish enable-module command
- publish page run command
- request import/export through `/bascorro_studio/request`
- subscribe result events
- subscribe live joint states

Suggested folder:

- `src/bascorro_studio/web/src/action_editor/ros/`

### 3. UI slices

Split UI into focused components.

Suggested components:

- `ActionEditorPage.jsx`
- `PageList.jsx`
- `PageHeaderForm.jsx`
- `StepList.jsx`
- `StepInspector.jsx`
- `JointGrid.jsx`
- `TimelinePreview.jsx`
- `RobotPreview.jsx`
- `YamlPanel.jsx`
- `HistoryBar.jsx`
- `ScratchRunPanel.jsx`

### 4. Backend boundary

Keep backend simple.

`apply_node.py` should remain the only place that mutates the binary action file for now. It should:

- accept import/export requests
- shell to `action_yaml.py`
- publish structured result payloads

If extended, add only small focused capabilities:

- validate request schema
- explicit `list_pages`
- explicit `read_current_action_file`
- optional dry-run validation

## Functional Spec

### Core v1 features

1. Load/export YAML from the active OP3 action file.
2. Browse pages by index and name.
3. Edit page header fields:
   - name
   - repeat
   - schedule
   - speed
   - accel
   - next
   - exit
4. Edit up to 7 steps per page.
5. Edit each step:
   - positions per OP3 joint
   - pause
   - time
6. Show live robot pose as reference.
7. Show 3D preview of edited pose.
8. Mirror left/right poses using OP3-specific joint rules.
9. Scratch-run current step or selected range via a temporary page.
10. Apply YAML back into the action binary through backend bridge.
11. Undo/redo and local draft recovery.

### Nice-to-have v2 features

1. Compare current step vs live pose.
2. Record pose from live robot into selected step.
3. Per-joint lock/freeze controls.
4. Multi-step interpolation preview.
5. Semantic tags for pages, for example:
   - init
   - getup
   - kick
   - goalkeeper
6. Action-page test history and notes.
7. Safer hardware run guardrails.

## Data Model

Use YAML as the editable document model in the frontend, but keep an internal normalized JS object.

Suggested normalized shape:

```yaml
meta:
  source_action_file: /path/to/motion_custom.bin
  joint_order:
    - id: 1
      name: r_sho_pitch
pages:
  - index: 1
    name: init_pose
    header:
      repeat: 1
      schedule: speed
      stepnum: 2
      speed: 32
      accel: 0
      next: 0
      exit: 0
      pgain: {}
    steps:
      - index: 0
        pause: 0
        time: 8
        positions:
          r_sho_pitch: 2048
          l_sho_pitch: 2048
```

Frontend editing should operate on normalized objects, then serialize back to YAML only at save/apply/export boundaries.

## Validation Rules

Validate early in the frontend and again before backend apply.

Required rules:

- page index in valid range
- unique page indexes
- max 7 steps per page
- step indexes normalized sequentially
- known OP3 joint names only
- header numeric ranges valid for byte-sized fields where applicable
- `stepnum` must match actual step count
- `schedule` must be `speed` or `time`
- scratch page index must not overwrite an important real page by accident

Hardware safety checks:

- warn before running on hardware if current module is not `action_module`
- warn before overwriting the action binary
- warn before using scratch page in low-numbered operational pages

## UX Direction

### Main workflow

1. Connect to ROS bridge.
2. Export current action file to YAML.
3. Select page.
4. Edit page header and steps.
5. Preview in 3D.
6. Scratch-run selected step or range.
7. Apply YAML.
8. Run actual page through `action_module`.

### Layout recommendation

Three-column desktop layout:

- left: page list
- center: step editor + timeline
- right: robot preview + run/apply tools

Mobile/tablet can collapse to tabs, but desktop is primary.

### Design principle

Optimize for fast repeated motion iteration, not generic dashboard aesthetics.

Important priorities:

- page/step selection must be fast
- joint editing must be dense and legible
- run/apply status must be obvious
- hardware-vs-simulation state must be obvious

## Refactor Plan

### Phase 1: stabilize

- freeze current ROS contract
- document action YAML format
- extract pure utility code from `ActionEditor.jsx`
- keep UI behavior mostly unchanged

### Phase 2: split editor

- create dedicated subcomponents
- move ROS logic into gateway hooks/services
- move history and persistence into isolated hooks
- reduce `ActionEditor.jsx` to orchestration only

### Phase 3: improve OP3 workflow

- add page categories/tags
- add better live-pose capture
- add stronger scratch-run flow
- add validation and hardware safety prompts

### Phase 4: optional backend cleanup

- add structured backend request types
- add dry-run validation mode
- add action-file metadata endpoint if needed

## Recommended First Implementation Slice

If the rebuild starts now, the first slice should be:

1. Extract OP3 action domain utilities out of `ActionEditor.jsx`.
2. Extract ROS bridge logic into a dedicated hook/service.
3. Split UI into:
   - page list
   - page/step inspector
   - robot preview
   - YAML panel
4. Keep `apply_node.py` and `action_yaml.py` unchanged for compatibility.

This gives a cleaner codebase quickly without breaking the existing OP3 runtime.

## Decision Summary

The safest clean rebuild is not to rewrite OP3 action execution. It is to:

- keep `op3_manager`
- keep `op3_action_module`
- keep binary compatibility through `action_yaml.py`
- rebuild the Bascorro Studio action editor as a thinner, OP3-specific frontend and backend bridge

That gets maintainability and better UX without destabilizing the robot stack.
