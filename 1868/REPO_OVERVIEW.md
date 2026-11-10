# Space Cookies Software Monorepo (FRC Team 1868)

A FIRST Robotics Competition robot-code monorepo built on **Bazel (bzlmod)** and
layered on top of **AOS** (the Realtime Robotics Group's event-loop framework,
vendored at `third_party/aos`). Actively developed; currently tracking `main`.

## Build System & Infrastructure

- **Bazel with bzlmod** (`MODULE.bazel`) is the sole build system, with a large,
  carefully pinned dependency graph:
  - **Robotics:** AOS, WPILib 2026.2.2, CTRE Phoenix 6 (26.x), Eigen, Ceres, OpenCV
  - **Toolchains:** LLVM/clang 22.1.8 (x86_64, aarch64, Darwin arm64), Rust, Go,
    Python 3.10/3.13, roboRIO ARM cross-compile
  - Extensive inline comments explain *why* each override/patch exists (the 2026
    WPILib/Phoenix migration, bindgen sysroot fixes, etc.)
- **Self-hosted dependency mirror** (`mirror.spacecookies.dev`, `tools/mirror_*`)
  so builds pull zero artifacts from the public internet — see `README.md`.
- `SETUP.md` contains onboarding instructions for student members (install
  VSCode, Bazel, SSH keys to the build server).

## Robot Code

### `offseason2026/` — the fullest robot program
- `drivetrain/` — swerve drivetrain with CTRE Tuner X generated constants
  (`generated/TunerConstants.*`), FlatBuffers schemas (`.fbs`) for
  goal/status/position/output, a writer, and tests.
- `super_loop/` — superstructure (intake + indexer) logic with constants,
  joystick input handling, and tests.
- `wpilib_interface.cc`, `robot.json` — the hardware/runtime glue.

### `workshops2026/cookie_loop/`
A teaching/starter version of an AOS event loop for onboarding students.

### `2026/vision/`
The vision subsystem: AprilTag-based aiming (`aimer.cc/.h`, `aim_goal.fbs`),
plus a large set of per-camera / per-Orin **calibration JSON files** (4 Jetson
Orins, many cameras, intrinsic and extrinsic calibration).

### `experimental/caitlincai/`
A personal scratch directory (just a `foo.txt`).

## Architecture Pattern

Everything follows the AOS idiom: subsystems communicate over FlatBuffer
messages on channels defined by `.fbs` schemas and wired up in `robot.json`.
Each subsystem is an event loop with goal/status/position/output messages,
giving a clean separation between control logic and the WPILib/CTRE hardware
interface.


## AprilTag

The AprilTag detection algorithm is a visual fiducial system used in robotics for real-time 3D pose estimation and localization. The pipeline converts grayscale images through decimation, adaptive thresholding, gradient-based quad fitting, and Hamming-distance decoding to output unique tag IDs and spatial coordinates.

Core Detection Pipeline

The standard AprilTag processing pipeline (evolving from Olson 2011 to AprilTag 3) consists of the following sequential stages:
• Image Preprocessing: The input RGB/color image is converted to grayscale. Optional Gaussian blur (quad_sigma) or decimation (quad_decimate) can be applied to reduce image size and speed up processing at the expense of distance or noise.
• Adaptive Thresholding: An adaptive binarization threshold is applied across local regions (e.g., 4 × 4 cells) to separate dark and light regions despite uneven lighting.
• Segment & Contour Extraction: Instead of raw line tracing that breaks on noisy pixels, the algorithm groups black and white pixels into clusters and extracts connected component contours to form candidate edge segments.
• Quad Fitting: Lines are fitted to candidate clusters to identify four distinct corner points forming a quadrilateral (quad). Subpixel corner refinement is performed by sampling image gradients around the corners.
• Tag Decoding & Error Correction: A homography matrix maps the quad back to a canonical grid. The internal bit pattern is extracted and compared against known tag families (such as 36h11) using XOR operations and Hamming distance thresholds to verify validity and correct bit errors.
• Pose Estimation: Using the known physical dimensions of the tag and camera intrinsic parameters (\(f_x, f_y, c_x, c_y\)), a Perspective-n-Point (PnP) or SVD-based solver computes the 3D position (X, Y, Z) and orientation (roll, pitch, yaw) relative to the camera.