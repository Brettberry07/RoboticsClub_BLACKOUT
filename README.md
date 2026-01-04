# BLACKOUT Robotics Codebase (2025–2026)

A modular, competition-ready VEX V5 robotics codebase built with PROS (C++) featuring robust odometry, path following, autonomous selection UI, and subsystem control. Designed for reliability, clarity, and fast iteration during matches.

---

## Highlights

- Auton Selection UI: In-bot screen selector with flexible paths and routines
- Odometry: Field-centric tracking using encoders/rotation sensors with gear ratio fixes
- Path Following: Curved and straight path support, tunable PID based follower
- Subsystems: Drivetrain (OOP), Intake, Pneumatics, PID controller utilities
- Analytics: Match and routine analytics hooks for performance insights
- Tests & Examples: Targeted tests for path following and odometry, plus example autonomous paths

## Tech Stack

- VEX V5 + PROS: Firmware and RTOS from the PROS ecosystem (C++)
- C++: Modern C++ with headers under `include/` and sources in `src/`
- LVGL (LittlevGL): Lightweight GUI components for on-robot display UI
- Build: `Makefile` based via PROS CLI, `compile_commands.json` for IDE tooling
- Assets & Paths: JSON path definitions under `paths/`

## Project Structure

- firmware/: Linker scripts for V5 builds
- include/: Public headers and subsystem APIs
  - pros/: PROS headers and C++ wrappers
  - liblvgl/: LVGL library
- src/: Implementation sources (autonomous, drivetrain, intake, odometry, path follower, etc.)
- paths/: Path JSON files (e.g., `straight_path.json`, `curved_path.json`)
- notebook/: Notes, images, and TODOs
- bin/icons/: UI icons and assets
- Docs (.md at root): Quick start and subsystem guides

Key files to explore:
- [include/autonSelector.hpp](include/autonSelector.hpp) – Autonomous selection interface
- [include/odometry.hpp](include/odometry.hpp) – Odometry types and API
- [include/pathFollower.hpp](include/pathFollower.hpp) – Path follower API
- [include/drivetrain_oop.hpp](include/drivetrain_oop.hpp) – Drivetrain abstraction
- [src/autonomous.cpp](src/autonomous.cpp) – Competition autonomous routines
- [src/pathFollower.cpp](src/pathFollower.cpp) – Path follower implementation
- [src/odometry.cpp](src/odometry.cpp) – Odometry update logic
- [src/tests.cpp](src/tests.cpp) – Aggregated test harness

## Quick Start (macOS)

Prerequisites:
- Python/pipx and PROS CLI installed (or use an existing setup)
- VS Code with C++ extensions recommended

Common commands:

```bash
# Build with PROS
pros build

# Upload firmware to the robot
pros upload

# Open a terminal to the V5 brain
pros terminal

# Generate compile commands for IDE tooling
pros build-compile-commands --no-analytics
```

For more details, see the inline comments in source files and subsystem headers.

## Autonomous Selection

The in-robot autonomous selector allows choosing routines pre-match via the display:
- UI built on LVGL and PROS `llemu`
- Select from predefined paths and strategies
- Integrates with autonomous runner in `src/autonomous.cpp`

Example implementation:
- [include/autonomousPathExample.hpp](include/autonomousPathExample.hpp)
- [src/autonomousPathExample.cpp](src/autonomousPathExample.cpp)

## Motion: Odometry & Path Following

Odometry:
- Field-centric position tracking using rotation/encoders
- Handles gear ratio adjustments and tracking wheel configurations

Path Following:
- Supports straight and curved paths via JSON definitions
- PID-based follower with tunables

Relevant files:
- [include/odometry.hpp](include/odometry.hpp), [src/odometry.cpp](src/odometry.cpp)
- [include/pathFollower.hpp](include/pathFollower.hpp), [src/pathFollower.cpp](src/pathFollower.cpp)
- Paths: [paths/straight_path.json](paths/straight_path.json), [paths/curved_path.json](paths/curved_path.json)

## Subsystems & Utilities

- Drivetrain (OOP): [drivetrain_oop.hpp](include/drivetrain_oop.hpp), [drivetrain_oop.cpp](src/drivetrain_oop.cpp)
- Intake: [intake.hpp](include/intake.hpp), [intake.cpp](src/intake.cpp)
- Pneumatics: [pneumatics.hpp](include/pneumatics.hpp), [pneumatics.cpp](src/pneumatics.cpp)
- PID Controller: [pidController.hpp](include/pidController.hpp), [pidController.cpp](src/pidController.cpp)
- Robot orchestration: [robot.hpp](include/robot.hpp), [robot.cpp](src/robot.cpp)

Sensors via PROS headers are available under [include/pros/](include/pros/): IMU, GPS, rotation, distance, optical, vision, and more.

## Configuration & Tuning

Common places to adjust behavior:
- Global config: [include/globals.hpp](include/globals.hpp), [src/globals.cpp](src/globals.cpp)
- PID gains: [include/pidController.hpp](include/pidController.hpp)
- Path definitions: [paths/](paths)
- Autonomous routines: [src/autonomous.cpp](src/autonomous.cpp)

Reference notes:
- Tracking wheel gear ratio considerations (see comments in odometry modules)
- Distance scaling fixes for sensor calibration
- Build configuration notes related to color types and multiple definition handling

## Development Workflow

- Branches: Work off `working`, merge into `main` after validation
- Build & run: Use PROS CLI commands from Quick Start
- IDE: Use `compile_commands.json` for IntelliSense (already generated in this repo)
- Tests: Targeted test files exist (e.g., [pathFollowerTests.cpp](src/pathFollowerTests.cpp), [odometryTests.cpp](src/odometryTests.cpp))

## Analytics

Competition analytics hooks and summaries:
- Enable logging during matches to monitor routine performance
- Review analytics integration points in [include/competitionAnalytics.hpp](include/competitionAnalytics.hpp) and [src/competitionAnalytics.cpp](src/competitionAnalytics.cpp)

## Documentation

This README is self-contained. Explore headers in [include/](include) and implementations in [src/](src) for detailed comments and usage patterns.

## Acknowledgements

Built by the BLACKOUT Robotics Club. Powered by PROS and LVGL in the VEX ecosystem.
