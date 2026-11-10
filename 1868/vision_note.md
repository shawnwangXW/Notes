# Vision Notes

## Table of Contents

- [AprilTag gradient-based quad fitting](#apriltag-gradient-based-quad-fitting)
  - [1. Collect boundary points](#1-collect-boundary-points-do_gradient_clusters)
  - [2. Sort around the center and check polarity](#2-sort-around-the-center-and-check-polarity-fit_quad-step-1)
  - [3. Weighted prefix sums](#3-weighted-prefix-sums-compute_lfps)
  - [4. Line fit](#4-line-fit-fit_line)
  - [5. Find corner candidates](#5-find-corner-candidates-quad_segment_maxima)
  - [6. Pick the best 4 corners](#6-pick-the-best-4-corners)
  - [7. Corners from line intersections](#7-corners-from-line-intersections)
  - [8. Edge refinement](#8-edge-refinement-refine_edges-on-by-default)
  - [Settings involved](#settings-involved-defaults-from-apriltag_detector_create)
  - [Notes about this repo](#notes-about-this-repo)
- [Adding a camera](#adding-a-camera)
  - [How it's wired today](#how-its-wired-today)
  - [Case A: swap a camera into an existing slot](#case-a-swap-a-camera-into-an-existing-slot)
  - [Case B: add a 5th camera](#case-b-add-a-5th-camera)
  - [Existing problems found](#existing-problems-found)
- [How localizer_main works](#how-localizer_main-works)
  - [Startup](#startup)
  - [The filter (HybridEkf)](#the-filter-hybridekf)
  - [Odometry input (HandleChassisSpeeds)](#odometry-input-handlechassisspeeds)
  - [Vision correction (HandleTarget, once per detected tag)](#vision-correction-handletarget-once-per-detected-tag)
  - [Resets (HandleControl)](#resets-handlecontrol)
  - [Outputs](#outputs)
  - [Settings in this repo](#settings-in-this-repo)
  - [Problems for this repo](#problems-for-this-repo)

## AprilTag gradient-based quad fitting

Code: the vendored AprilTag 3 library, mainly `third_party/aos/third_party/apriltag/apriltag_quad_thresh.c` (`fit_quad` and the functions it calls), plus `refine_edges` in `apriltag.c`.

The code never runs a typical edge detector. Image gradients do three jobs:
- record which side of each edge point is white,
- weight points in the line fits,
- shift the fitted edges to sub-pixel accuracy at the end.

By the time quad fitting starts, the image has been decimated and thresholded so every pixel is black, white or "unsure", and union-find has grouped pixels into connected black regions and white regions.

### 1. Collect boundary points (`do_gradient_clusters`)

- Scan for pairs of neighbouring pixels where one is black and the other is white. Both regions must have at least 25 pixels.
- Record the point halfway between them, using doubled coordinates so the halves stay integers.
- Each point goes into a cluster keyed by the pair of region IDs (this black region, that white region). Keying by the pair stops two nearby edges from merging into one cluster.
- Each point also stores `gx, gy` (-255, 0 or +255), pointing toward the white pixel: the gradient direction at that point.

So one cluster is the outline where one particular black region meets one particular white region. A tag's black border is one such cluster.

### 2. Sort around the center and check polarity (`fit_quad`, step 1)

- Take the center of the cluster's bounding box, nudged by a tiny offset so no point sits at exactly the same angle as another.
- Sort the points by angle around that center. To avoid `atan2`, it uses a cheap stand-in: quadrant plus a slope (`dy/dx`).
- Add up `dot = Σ (p − center) · (gx, gy)`. Positive means white is outside: a normal tag (black border, white surround). Negative means a reversed border. Clusters with the wrong polarity are dropped.

After sorting, the points go once around the quad's perimeter in order. Finding the quad now means cutting that circular list into 4 runs, one per edge.

### 3. Weighted prefix sums (`compute_lfps`)

For each point, measure the gradient magnitude in the grayscale image and set the weight `W = |∇I| + 1`. Keep running totals of `W`, `W·x`, `W·y`, `W·x²`, `W·xy` and `W·y²` along the sorted list.

- Points on sharp, high-contrast edges count for more than blurry or noisy ones.
- A line can be fitted to any run `[i0, i1]` in O(1) time by subtracting two running totals (wrap-around runs too). This makes the corner search affordable.

### 4. Line fit (`fit_line`)

A weighted principal-component fit:

- From the totals, compute the weighted centroid and the 2×2 covariance matrix.
- The line passes through the centroid along the direction of greatest spread. The function returns the line's normal.
- The smaller eigenvalue is the mean squared distance of the points from the line (`mse`). `err = N · mse` is the total error.

### 5. Find corner candidates (`quad_segment_maxima`)

- For every point `i`, fit a line to a window of ±`ksz` points around it (`ksz = min(20, sz/12)`). Along a straight edge the error is near zero; at a corner the window bends and the error jumps.
- Smooth the error curve with a small Gaussian (σ = 1), wrapping around because the list is circular.
- Take the local maxima and keep the strongest `max_nmaxima` (default 10).

### 6. Pick the best 4 corners

- Try every ordered set of 4 maxima (210 combinations with 10 candidates).
- Fit the 4 edges. Reject if any edge has `mse > max_line_fit_mse` (default 10), or if the first two adjacent edges are within 10° of parallel (`cos_critical_rad`).
- Keep the combination with the smallest total error, then require total error / point count to be under the mse limit.

### 7. Corners from line intersections

Back in `fit_quad`, the 4 edges are refitted and each corner is the intersection of two neighbouring edges (a 2×2 linear solve). The quad is rejected if:
- two lines are nearly parallel (`|det| < 0.001`)
- its area is under 95% of `tag_width²`
- any corner angle is outside about 10° to 170°, or the corners wind the wrong way (not convex)

The corners come from fitted lines, not single pixels, so they're already sub-pixel and robust to a few noisy pixels.

### 8. Edge refinement (`refine_edges`, on by default)

The quad was found on the decimated image (`quad_decimate = 2.0` by default), so the corners are scaled back up and refined on the full-resolution grayscale image:

- For each edge, take `max(16, length/8)` sample points along it, deliberately avoiding the corners (the least reliable part).
- At each sample, step along the edge's normal from −(decimate+1) to +(decimate+1) pixels in 0.25-pixel steps, comparing intensities 1 pixel either side of each step.
- Throw away readings where intensity changes in the wrong direction (from the border polarity). Weight the rest by the squared intensity difference. The weighted average position is the sub-pixel edge location at that sample.
- Fit a new line through the refined samples and recompute the 4 corners as intersections.

This step accounts for most of the final corner accuracy, which matters directly for pose: `estimate_tag_pose` (PnP) error grows with corner error, especially at long range.

After this the quad goes on to decoding: a homography maps it to the tag grid, the bits are read and checked by Hamming distance against the tag family.

### Settings involved (defaults from `apriltag_detector_create`)

| Parameter | Default | Effect |
|---|---|---|
| `quad_decimate` | 2.0 | Quads found at half resolution; also sets the refinement search range |
| `refine_edges` | true | Step 8; leave it on when decimating |
| `qtp.max_nmaxima` | 10 | Corner candidates considered |
| `qtp.max_line_fit_mse` | 10.0 | How straight each edge must be |
| `qtp.cos_critical_rad` | cos(10°) | Smallest allowed corner angle |
| `qtp.min_white_black_diff` | 5 | Thresholding contrast floor, before this stage |

`min_cluster_pixels` exists too, but the clustering code hard-codes the 25-pixel region size and 24-point cluster minimum.

### Notes about this repo

- The robot's vision doesn't run this CPU code for detection. `2026/vision` deploys `@aos//frc/vision:apriltag_detector`, which needs an NVIDIA GPU. It uses 971's CUDA re-implementation in `third_party/aos/frc/orin/apriltag.cc`, which runs these stages on the GPU (`apriltag::FitQuads` is called with the same `cos_critical_rad` and related settings) and still uses the upstream library's detector struct and pose code. Only the CPU reference was read closely, not the CUDA kernels, so treat them as equivalent in algorithm, not necessarily identical bit for bit.
- `REPO_OVERVIEW.md` says sub-pixel refinement samples gradients "around the corners". It actually samples along the edges, deliberately skipping the corners, and recomputes the corners as intersections of the refined lines.

## Adding a camera

The vision system is built for exactly 4 cameras on one Orin. Swapping a camera into one of the 4 existing slots only needs config and calibration changes. Adding a 5th camera also means changing several hard-coded `4`s in the vendored AOS code. (From reading the code; nothing built or run.)

### How it's wired today

- One AOS node, `orin` (hostnames `orin-[0-9]*-1`), with cameras 0–3. In calibration file names like `calibration_orin-1868-2_cam-26-10_...`, the `2` is the **camera number**, not an Orin number. `ARCHITECTURE.md` and `REPO_OVERVIEW.md` both describe "4 Orins per robot", which is wrong.
- Each camera N has three applications in `2026/vision/single_node_camera_config.json`, wired by renaming `/camera` to `/cameraN` through `maps`:
  - `camera_readerN`, reading `--viddevice=/dev/video{a,b,c,d}`
  - `turbojpeg_decoderN`
  - `apriltag_detectorN --channel /cameraN/gray`
- Each camera needs a calibration entry (`node_name: "orin"`, `camera_number: N`, intrinsics, `fixed_extrinsics`) in every robot's block of `constants.jinja2.json`.
- `apriltag_detector` gets the camera number by parsing one character of the channel name (`substr(7, 1)`), so camera numbers 0–9 work.

### Case A: swap a camera into an existing slot

1. Plug it in and check it shows up as the right `/dev/videoX`. The `videoa`–`videod` names come from the Orin's OS image, not this repo (no udev rule found in the tree). On the Orin, `ls -l /dev/video*` and `udevadm info /dev/videoa` show the mapping.
2. Calibrate intrinsics on the Orin with the deployed `intrinsics_calibration` (`--channel=/cameraN`, `--camera_id=26-XX`, `--calibration_folder`, `--base_intrinsics`; check `--help` for the full set).
3. Save as `2026/vision/constants/glued/<robot>/calibration_orin-1868-N_cam-26-XX_MM-DD-26.json` and point that robot's `{% include %}` at it.
4. Re-run extrinsics (`README.md` procedure) so the file gets `fixed_extrinsics`. The localizer refuses to start without them.
5. Build and deploy (below).

### Case B: add a 5th camera

#### 1. Hardware

- Each camera streams MJPEG at 1600×1304, with channels sized for up to 125 Hz. Check USB bandwidth (ideally a separate controller or hub port) and Orin load (`hardware_monitor` publishes on `/hardware_monitor`). Each camera adds three more processes, including a GPU detector.
- Make sure it gets a stable name such as `/dev/videoe`, using the same mechanism the OS image uses for the first four.

#### 2. `single_node_camera_config.json`

- Copy the seven `/camera3` and `/camera3/gray` channel blocks to `/camera4`: `CameraImage`, `CameraStreamSettings`, gray `CameraImage`, `TargetMap`, `foxglove.ImageAnnotations`, `TurboJpegDecoderStatus`, `Visualization`.
- Copy `camera_reader3` (with `/dev/videoe`), `turbojpeg_decoder3` and `apriltag_detector3` to camera 4.

#### 3. Calibration

- Calibrate intrinsics as in case A; save as `calibration_orin-1868-4_...json` with `"camera_number": 4`.
- Add the include to **every** robot block in `constants.jinja2.json` that runs this config. With the localizer change below, any robot without a camera-4 entry crashes the localizer.

#### 4. AOS code with hard-coded 4 cameras

All in `third_party/aos/frc/vision/`:

| File | What to change |
|---|---|
| `swerve_localizer/localizer.h` | `kNumCameras = 4` → 5 |
| `swerve_localizer/localizer.cc` | Add `"/camera4/gray"` to `kDetectionChannels` |
| `swerve_localizer/localizer_replay.cc` | Add `"camera4"` to the camera list |
| `network_tables_publisher.cc` | `resize(4)` / `i < 4` loop, plus the per-camera NetworkTables topics |
| `field_side_exposure_adjuster.cc` | Add a `/camera4` `CameraStreamSettings` sender |
| `swerve_localizer/www/field_handler.ts` | Add to `CAMERAS` and `CAMERA_COLORS` (debug field view) |
| `calibrate_multi_cameras_lib.cc` | Extrinsics ring order is `0, 3, 2, 1` (adjacent entries are physical neighbours). Insert 4 between its real neighbours. |

`third_party/aos` is a git subtree of RTR's AOS, so these edits diverge from upstream (`diff-from-upstream.sh` shows the difference). Cleaner: make the count come from the calibration list or config instead of changing 4 to 5, and send that upstream. Otherwise every robot must have exactly 5 cameras.

#### 5. Extrinsics

Follow the two-tag procedure in `2026/vision/README.md`, making sure camera 4 and each of its neighbours see the tags, then run `//2026/vision:extrinsics_calibration` on the log. That writes `fixed_extrinsics` for all cameras.

#### 6. Build and deploy

```
./tools/bazel build --config=arm64 //2026/vision:constants.json   # validates calibrations
./tools/bazel run --config=arm64 -c opt //2026/vision:download_stripped -- pi@10.18.68.101
```

On the Orin, `aos_starter status` should show `camera_reader4`, `turbojpeg_decoder4` and `apriltag_detector4` running; `aos_dump /camera4/gray frc.vision.TargetMap` should show detections; `localizer_main` should start without "Missing a camera calibration".

### Existing problems found

- The `stick` robot has two `camera_number: 0` entries and no camera 1. The `test` robot has numbers 0, 2, 3, 3. The localizer stops with `Got multiple calibrations` or `Missing a camera calibration` on both.
- Several files under `constants/glued/` (not in `stella/` or `supernova/`) have no `fixed_extrinsics`, so `stick` and `testboard` would also fail the localizer's extrinsics check.
- `glued/calibration_orin-1868-3_cam-26-11_02-01-26.json` has `camera_id: "25-11"`, which doesn't match its file name.

## How localizer_main works

`localizer_main` is the Orin process that combines the roboRIO's odometry with the AprilTag detections from all 4 cameras into one field pose (x, y, θ). It uses a small EKF that keeps a history of past states. Code: `third_party/aos/frc/vision/swerve_localizer/` (`localizer_main.cc`, `localizer.{h,cc}`, `hybrid_ekf.h`). (Read, not run.)

### Startup

```cpp
frc::constants::WaitForConstants<CameraConstants>(...);  // calibrations
frc::constants::WaitForConstants<TargetMap>(...);        // field tag poses
aos::ShmEventLoop event_loop(...);
Localizer localizer(&event_loop);
event_loop.Run();
```

Blocks until `vision_constants_sender` and `field_map_constants_sender` have published, then the `Localizer` constructor sets up:

- `MakeCameras`: loads each camera's `fixed_extrinsics` (camera position and angle on the robot). Requires exactly one calibration for each of cameras 0–3, otherwise stops with `Got multiple calibrations` or `Missing a camera calibration`.
- `GetTargetLocations`: tag ID → tag's field position, from the `.fmap` file.
- Watchers on `/camera{0..3}/gray` `TargetMap`, `/drivetrain` `ChassisSpeeds`, `/drivetrain` `LocalizerControl`.
- A 20 ms timer calling `SendOutput()`.
- Real-time priority 10. On loop start, the EKF is reset to the origin with a "typical" covariance so the first corrections don't make the pose jump.

### The filter (HybridEkf)

- State: `[x, y, θ]` on the field. Input: `[vx, vy, ω]` in field coordinates.
- Keeps the last 200 states and inputs (`kSaveSamples`), so a camera frame is compared against where the robot was **when the frame was captured**.
- To save CPU, a correction uses the capture-time state to compute the expected measurement but applies the update at the current time, rather than replaying the whole history.

### Odometry input (HandleChassisSpeeds)

These come from the roboRIO through `network_tables_swerve_client` on the Orin, which listens on UDP port 4647 for 7 doubles `[x, y, θ, vx, vy, ω, publish_time]` and republishes them as `/drivetrain` `Pose2d` and `ChassisSpeeds`.

For each speeds message:

1. Rotate `(vx, vy)` from robot to field coordinates using the RIO's heading.
2. **Heading reset**: if the EKF's θ differs from the RIO's by more than 0.4 rad in teleop (1.5 rad in auto), reset the EKF. Auto keeps x and y; teleop uses a running average of recent camera poses for x and y. θ comes from the RIO in both cases (counted in `heading_resets`).
3. Step the filter forward with the previous speeds up to now.
4. **Heading follows the gyro**: the change in θ from integrating ω is replaced with the change in the RIO's heading (a comment says ω wasn't reliable enough). In practice cameras mainly correct x and y.
5. Store the new speeds for the next step.

### Vision correction (HandleTarget, once per detected tag)

1. Reject the frame if its capture timestamp is in the future.
2. Reject tags not in the field map. "Deweighted" tag IDs (every ID except 2–5, 8–11, 18–21, 24–27) are still used (`--always_use_extra_tags`), but with noise × 1.5.
3. Camera field pose = `H_field_target · H_camera_target⁻¹`. Removing the camera-to-robot transform (ignoring camera pitch and roll) gives the robot pose this tag implies.
4. Rejection checks (each counted per camera in `Status`):

| Reason | Condition |
|---|---|
| `IMAGE_TOO_OLD` | No EKF history at the capture time |
| `HIGH_POSE_ERROR` | `pose_error > 1e-5` |
| `HIGH_POSE_ERROR_RATIO` | `pose_error_ratio > 0.4` |
| `HIGH_DISTORTION` | `distortion_factor > 1000` |
| `ROBOT_TOO_FAST` | Auto only, speed > 5 m/s |
| `HIGH_IMPLIED_YAW_ERROR` | Tag-implied camera heading vs. expected from RIO heading > 10° (auto) or `--max_implied_teleop_yaw_error` (teleop) |
| `HIGH_DISTANCE_TO_TARGET` | > 5 m |

5. Update the filter. Two modes:
   - **Default, heading/distance/skew**: measurement is the tag's bearing, range and relative angle from the camera, with an approximate Jacobian. Noise starts at (0.03, 0.25, 0.15) × 2, then grows with distance², distortion, the deweight factor and robot speed.
   - **`--do_xytheta_corrections`** (used in this repo): measurement is the implied robot `(x, y, θ)` directly (`XyzCorrector`). Noise (0.2 m, 0.2 m, 0.5 rad), scaled by distance², deweight factor and distortion. Unlike the default, robot speed doesn't increase the noise.
6. Write debug info (implied pose, expected vs. actual, correction applied, accept or reject) to `/cameraN/gray` `Visualization` for the web field view.

### Resets (HandleControl)

A `LocalizerControl` on `/drivetrain` sets the state to a given `(x, y, θ)`, optionally keeping the current θ (for placing the robot before a match). The channel is in the config, but nothing in the deployed vision set was found to send it.

### Outputs

- `/localizer` `LocalizerOutput` every 20 ms (50 Hz, though the channel allows 500 Hz): `x, y, θ`, accepted image count, heading resets, speeds count. Used by `aimer`; `network_tables_swerve_client` also sends it back to the roboRIO over UDP port 4648 as `[x, y, θ, …, image_accepted_count]`, shifted to field-corner coordinates.
- `/localizer` `Status` on every image or speeds update: state, 3×3 covariance, per-camera accept/reject statistics.

### Settings in this repo

From `single_node_camera_config.json`:

```
localizer_main --max_implied_teleop_yaw_error=90 --do_xytheta_corrections
```

- Teleop yaw-error limit raised from 30° to 90°, so in teleop almost no tag is rejected for heading disagreement.
- Uses the simpler x/y/θ correction mode.

### Problems for this repo

- **The roboRIO side doesn't exist in `offseason2026`.** The localizer needs the RIO to send 7-double UDP packets to port 4647, and `network_tables_swerve_client` also expects AdvantageKit NetworkTables topics for Driver Station state. `wpilib_interface.cc` does neither. Without them, `HandleChassisSpeeds` returns early (no RIO pose), the filter never moves forward in time, and camera corrections only work against a pose that isn't moving. Presumably the 2026 competition robot code that sent this data is a separate Java/AdvantageKit program not in this repo.
- The 4-camera limit and the calibration problems (see "Adding a camera") apply here: `MakeCameras` crashes for the `stick` and `test` robot configs.
