# Vision Subsystem Architecture (`2026/vision/`)

An AprilTag-based localization and shooter-aiming pipeline that runs entirely on
NVIDIA Jetson **Orin** coprocessors (not the roboRIO), built on the AOS
event-loop / pub-sub framework.

Team 1868's own code here is small — `aimer`, `aimer_forwarder`, the `AimGoal`
schema, and the calibration/config. The heavy lifting (camera reading,
decoding, AprilTag detection, localization) comes from reusable
`@aos//frc/vision` binaries deployed alongside it.

## Deployment Target

A single AOS node named `orin` (hostnames `orin-[0-9]*-1`), ARM64 only
(`robot_downloader` with `target_type = "orin"`). Each robot runs **4 cameras
per Orin**. The config is `single_node_camera_config.json` — it describes one
Orin's view of the world; multiple Orins (4 per robot, per the calibration
files) each run their own copy.

## Per-Camera Pipeline (×4: camera0–camera3)

Each camera flows through a fixed chain of AOS applications, wired by channel
name with `maps` that rename a generic `/camera` to `/cameraN`:

```
camera_readerN      /dev/video{a,b,c,d} ──▶ /cameraN  CameraImage (MJPEG, 125 Hz, PIN/zero-copy)
        │
turbojpeg_decoderN  ──▶ /cameraN/gray  CameraImage (MONO8, 125 Hz, NOT_LOGGED, zero-copy PIN)
        │
apriltag_detectorN  --use_neon --image_format=MONO8
        │           ──▶ /cameraN/gray  TargetMap (125 Hz)
        │           ──▶ foxglove.ImageAnnotations (debug overlay)
        ▼
localizer_main      fuses all 4 cameras' TargetMaps
                    ──▶ /localizer  LocalizerOutput + Status (500 Hz)
```

Key performance choices visible in the channels:

- `"read_method": "PIN"` with `num_readers` on the image channels — shared-memory
  zero-copy image passing (no serialization of ~4 MB frames).
- Decoded gray images are `"logger": "NOT_LOGGED"` with a short 100 ms storage
  duration — transient, not worth logging.
- AprilTag detection uses NEON SIMD (`--use_neon`) and an age cutoff
  (`--max_image_age_ms=55`) to drop stale frames.

## Aiming Layer (team's own code)

Once the localizer produces a robot pose, two small team-written binaries take
over.

### `aimer` (`aimer.cc` / `aimer.h`, `RobotAimer`)

- Watches `/localizer` `LocalizerOutput`. On every pose update it fetches the
  latest `JoystickState` (`/frc`, for alliance color), `ChassisSpeeds` and
  `Pose2d` (`/drivetrain`).
- Picks a **target pose** based on alliance and field position: either the
  alliance **hub** (if past a "bump" x-threshold) or a **shuttle target** (with
  a y-sign flip so the target mirrors to the robot's side of the field). Hub /
  shuttle field coordinates are hard-coded relative to field center, with
  `fieldwidth` / `fieldlength` pulled from a `TargetMap` constants channel.
- Feeds a `RobotState` (pose + field-relative velocity from chassis speeds
  rotated by heading) into AOS's shared
  `frc::control_loops::aiming::AimerGoal` "shoot-on-the-fly" solver. It runs the
  solver **twice**: once for a rough distance, then picks a `shot_velocity` from
  distance bands (a lookup table) and re-solves for a refined goal.
- Publishes an `AimGoal` on `/aimer`: turret `theta` / `omega`, measured +
  virtual (motion-compensated) `distance`, and absolute `target_x` / `target_y`.
  Runs at RT priority 2.

### `aimer_forwarder` (`aimer_forwarder.cc`)

- Watches `/aimer` `AimGoal` and ships the 4 doubles
  (`theta, omega, distance, virtual_shot_distance`) to the **roboRIO over raw
  UDP** (default host `roborio`, port `4649`) — this is how the Orin-computed
  aim gets back to the real-time robot controller. Runs at RT priority 1.
- Tracks send failures and emits a `UdpStatus` health message (`SEND_FAILURE`
  fault) on `/aimer` once per second.

## Calibration & Constants

- `constants.jinja2.json` is a **Jinja2 template** that `{% include %}`s
  per-camera calibration JSON, keyed by `team` + `robot_name` (`test`, `bot1`,
  `stella`, `supernova`, `stick`, `testboard`). Each named robot maps its 4
  Orin/camera calibrations plus default stream settings (resolution, exposure,
  gain).
- The build runs it through `jinja2_template` → `camera_constants_json` (a
  validator from `@aos//frc/vision:validator.bzl`) → `constants.json`, which is
  deployed and served at runtime by `vision_constants_sender` on the
  `/constants` channel. The naming
  `calibration_orin-1868-N_cam-26-XX_MM-DD-26.json` encodes Orin index, camera
  serial, and calibration date.
- **Extrinsics** (camera-to-camera / robot geometry) are solved offline from a
  recorded log via the `extrinsics_calibration` wrapper around
  `@aos//frc/vision:calibrate_multi_cameras`; see `README.md` for the
  two-AprilTag collection procedure.

## Debug / Observability

- `foxglove_websocket` + `web_proxy` serve a live web UI (ICE ports 5800–5810);
  detectors publish `foxglove.ImageAnnotations` overlays.
- `localizer_logger` and `image_logger` record logs on the Orin for the
  extrinsics solver and replay (`localizer_replay.sh`).
- `hardware_monitor` / `filesystem_monitor` report Orin health.

## Overall Data Flow

```
4× camera → decode → AprilTag → localizer_main → /localizer (robot pose)
                                                      │
                                                      ▼
                            aimer (alliance + target + shoot-on-the-fly solver)
                                                      │  /aimer AimGoal
                                                      ▼
                            aimer_forwarder ──UDP:4649──▶ roboRIO
```

Everything is decoupled through named FlatBuffer channels, so each stage is an
independently-restartable `aos_starter` application and logs can be replayed
deterministically offline.
