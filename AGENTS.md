# Repository Guidelines

## Project Structure & Module Organization
The workspace follows the ROS 2 layout: active packages live in `src/`, generated files land in `build/`, installed artifacts go to `install/`, and runtime logs collect in `log/`. Driver- and math-level libraries (`src/DynamixelSDK`, `src/ROBOTIS-Math`, `src/ROBOTIS-Utility`) should stay slim and reusable. Robot behaviors and demos live in `src/ROBOTIS-OP3-Demo`, while simulator assets and launchable worlds live in `src/ROBOTIS-OP3-Simulations`. Perception modules such as `src/face_detection` own their `config/`, `launch/`, and `rviz/` directories; follow the same pattern when adding new packages.

## Build, Test, and Development Commands
- `colcon build --merge-install --symlink-install`: build every package; re-run after touching shared messages or math libraries.
- `source install/setup.bash`: refresh your environment with the latest message/service definitions before running nodes.
- `colcon build --packages-select op3_manager`: limit the build scope for faster inner loops while touching a single module.
- `colcon test --event-handlers console_direct+ --packages-select op3_ball_detector_msgs`: execute available `ament_cmake_gtest` suites with verbose logs.
- `roslaunch face_detection face_detection.launch` (ROS 1 bridge) or `ros2 launch op3_demo op3_demo.launch.py`: start canonical Webots/OP3 demos; set `WEBOTS_HOME` if the simulator is outside the default path.

## Coding Style & Naming Conventions
C++ nodes follow the ROS 2 style used in `op3_ball_detector`: 2-space indentation, `CamelCase` classes, and `snake_case` methods or variables. Keep headers under `include/<package>` with matching namespaces, expose parameters via `declare_parameter`, and keep constants uppercase snake case. Prefer `.hpp` for template-heavy code and guard new files with Apache-2.0 headers. Python utilities (e.g., configuration generators in `face_detection/cfg/`) should remain PEP 8 compliant with 4-space indentation; run `ament_flake8` or `ament_uncrustify` locally before pushing.

## Testing Guidelines
Use `ament_add_gtest` or `ament_add_pytest_test` inside each package’s `CMakeLists.txt` and place sources under `test/` named `test_<feature>.cpp` or `test_<feature>.py`. Provide launch-based regression tests for nodes interacting with Webots by recording bag files in `test/data/` and replaying them in CI. Keep new logic at or above roughly 70 % statement coverage and document how to reproduce hardware-in-the-loop checks in package READMEs. Run `colcon test` before every PR and fix or isolate flaky tests with clear TODOs.

## Commit & Pull Request Guidelines
Follow the existing log style: concise, present-tense summaries (`"update editor webots"`) prefixed with the touched subsystem when helpful (`"op3_demo: add head scan"`). Reference related issue IDs in the body and describe observable behavior changes plus safety considerations (joint limits, camera calibration, launch-time parameters). PRs should include what/why sections, reproducible commands (`colcon build`, launch invocation), screenshots or bag excerpts for UI/simulation deltas, and confirmation that lint and tests pass in a sourced environment.
