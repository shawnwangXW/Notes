# Shawn's Notes

## Table of Contents

- [Shawn's Notes](#shawns-notes)
  - [Table of Contents](#table-of-contents)
  - [Pneumatics](#pneumatics)
    - [Components](#components)
    - [Single vs. double solenoid](#single-vs-double-solenoid)
    - [What it's used for](#what-its-used-for)
    - [Pros and cons vs. motors](#pros-and-cons-vs-motors)
    - [In code](#in-code)
  - [CTRE Tuner X](#ctre-tuner-x)
    - [What it does](#what-it-does)
    - [Connecting](#connecting)
  - [CANivore](#canivore)
    - [Why teams use it](#why-teams-use-it)
    - [How it works](#how-it-works)
    - [Hardware notes](#hardware-notes)
  - [roboRIO](#roborio)
    - [What's inside](#whats-inside)
    - [Ports and I/O](#ports-and-io)
    - [Its role on the robot](#its-role-on-the-robot)
    - [In this repo (AOS)](#in-this-repo-aos)
  - [Differential drivetrain](#differential-drivetrain)
    - [How it moves](#how-it-moves)
    - [Hardware](#hardware)
    - [Compared with swerve (what this robot uses)](#compared-with-swerve-what-this-robot-uses)
    - [In this repo](#in-this-repo)
  - [Bringing up the robot (offseason2026 on the roboRIO)](#bringing-up-the-robot-offseason2026-on-the-roborio)
    - [1. Flash the roboRIO (once per RIO)](#1-flash-the-roborio-once-per-rio)
    - [2. One-time AOS setup on a freshly flashed RIO](#2-one-time-aos-setup-on-a-freshly-flashed-rio)
    - [3. Build and deploy](#3-build-and-deploy)
    - [4. Start it](#4-start-it)
    - [5. Before enabling](#5-before-enabling)
  - [Dependency mirror](#dependency-mirror)
    - [1. URL rewriting (http\_archive, http\_file, toolchains, crates, ...)](#1-url-rewriting-http_archive-http_file-toolchains-crates-)
    - [2. Bazel module registry (bzlmod)](#2-bazel-module-registry-bzlmod)
    - [3. Python wheels](#3-python-wheels)
    - [Remote builds](#remote-builds)
    - [Discrepancies found](#discrepancies-found)
    - [When a build fails on a new dependency](#when-a-build-fails-on-a-new-dependency)
  - [IMU and wheel sensors (offseason2026)](#imu-and-wheel-sensors-offseason2026)
    - [Hardware](#hardware-1)
    - [How the data reaches AOS](#how-the-data-reaches-aos)
    - [What's missing](#whats-missing)
    - [One thing to check](#one-thing-to-check)
  - [Multi-process vs. multi-thread](#multi-process-vs-multi-thread)
    - [Multi-process: the AOS design](#multi-process-the-aos-design)
    - [Multi-threaded inside `wpilib_interface`](#multi-threaded-inside-wpilib_interface)
    - [In short](#in-short)
  - [AOS vs. ROS 2](#aos-vs-ros-2)
    - [Concept mapping](#concept-mapping)
    - [Key differences](#key-differences)
      - [1. Transport](#1-transport)
      - [2. Static vs. dynamic system shape](#2-static-vs-dynamic-system-shape)
      - [3. Real time](#3-real-time)
      - [4. Testing and replay](#4-testing-and-replay)
      - [5. Ecosystem](#5-ecosystem)
      - [6. Platform and build](#6-platform-and-build)
      - [7. Learning curve](#7-learning-curve)
    - [For this repo specifically](#for-this-repo-specifically)
  - [Where the main functions are](#where-the-main-functions-are)
    - [On the roboRIO (offseason2026)](#on-the-roborio-offseason2026)
      - [Control loop: `offseason2026/drivetrain/drivetrain_main.cc`](#control-loop-offseason2026drivetraindrivetrain_maincc)
      - [Hardware process: `offseason2026/wpilib_interface.cc` (line 126)](#hardware-process-offseason2026wpilib_interfacecc-line-126)
    - [On the Orin (2026/vision)](#on-the-orin-2026vision)
    - [What's not a robot program](#whats-not-a-robot-program)
  - [Code base issues](#code-base-issues)
    - [Build and test results](#build-and-test-results)
    - [Serious: the robot won't work as intended](#serious-the-robot-wont-work-as-intended)
    - [Medium](#medium)
    - [Low](#low)
  - [Practical Basel](#practical-basel)
  - [Basel Tutolrial](#basel-tutolrial)

## Pneumatics

Pneumatics is using compressed air to make things move. In FRC, it's one of the two main ways to actuate mechanisms, along with electric motors.

### Components

- **Compressor**: an onboard electric pump that pressurizes air, usually to about 120 psi.
- **Storage tanks**: hold the compressed air so you have a reserve for repeated actuations.
- **Pressure switch or sensor**: tells the control system when to turn the compressor on or off.
- **Regulator**: lowers the stored pressure to a working pressure (60 psi max for actuators under FRC rules).
- **Solenoid valves**: electrically controlled valves that send air to an actuator.
  - *Single solenoid*: one coil; springs back when power is cut.
  - *Double solenoid*: two coils; stays in its last position.
- **Cylinders (pistons)**: the actuators. Air pushes a rod out or pulls it in.
- **Pneumatics hub**: the CTRE PCM or REV PH. It sits on the CAN bus, drives the solenoids, and controls the compressor.

### Single vs. double solenoid

A single solenoid valve has one electric coil and a spring; a double (dual) solenoid valve has two coils and no spring. In WPILib these are `Solenoid` and `DoubleSolenoid`.

| | Single solenoid | Double solenoid |
|---|---|---|
| Coils | 1, plus a return spring | 2, no spring |
| Hub channels used | 1 | 2 (forward and reverse) |
| Coil powered | Valve shifts; cylinder extends (or retracts, depending on plumbing) | Pulse "forward" or "reverse" to shift the valve that way |
| Power removed | Spring returns the valve; cylinder goes back to its default position | Valve stays where it was; cylinder holds its last position |
| On robot disable or brownout | Mechanism goes to its spring default | Mechanism stays put (can't drift unless air leaks out) |
| WPILib | `Solenoid.set(true/false)` | `DoubleSolenoid.set(kForward / kReverse / kOff)` |

- **Use a single solenoid** when the mechanism needs a safe default position, e.g. an intake that should retract when disabled, or a brake that should engage. It also uses fewer hub channels, which matters on a crowded hub.
- **Use a double solenoid** when the mechanism must hold position with power off, e.g. a gear shifter or climber latch, so it doesn't flip when the robot disables at the end of a match or browns out.

Code notes:

- With a `DoubleSolenoid`, `kOff` turns both coils off, but the valve stays in its last position. Most teams call `set(kForward)` or `set(kReverse)` and never need `kOff`.
- WPILib's `DoubleSolenoid` reports an error if both channels are given the same number.
- Both kinds need air pressure to move anything. With an empty tank, nothing moves no matter what the valve does.

### What it's used for

- Two-position mechanisms: extending or retracting an intake, opening or closing a claw, shifting gearbox gears, deploying a climber or brake.
- Fast, strong, simple motion where you only need "in" or "out", not precise positioning.

### Pros and cons vs. motors

- **Pros**: simple to control (just on or off), strong for their size and weight, and they hold position without drawing current.
- **Cons**: usually only two positions, a limited air supply, extra weight from the compressor and tanks, and leaks to deal with.

### In code

WPILib provides `Solenoid`, `DoubleSolenoid`, `Compressor`, and `PneumaticHub` / `PneumaticsControlModule` classes. You set a solenoid to forward, reverse, or off, and the hub opens the matching valve.

## CTRE Tuner X

CTRE Tuner X (Phoenix Tuner X) is the desktop and mobile app from Cross The Road Electronics (CTRE) for setting up and debugging their FRC hardware. It runs on Windows, Android, iOS and macOS, and works with Talon FX (Kraken and Falcon motors), Talon SRX, Victor SPX, CANcoder, Pigeon 2, CANdle and CANivore.

### What it does

- **Device discovery**: lists every CTRE device on the CAN bus, including ones on a CANivore, with ID, model and firmware version.
- **Firmware updates**: flashes new firmware to devices, one at a time or in batches.
- **CAN ID and naming**: changes device IDs to fix ID conflicts and gives devices readable names.
- **Configuration**: reads and writes settings such as current limits, neutral mode (brake or coast), inverts, PID gains, sensor sources and soft limits.
- **Self-test snapshots**: shows live status, faults, voltage, temperature, position and velocity for one device.
- **Control**: drives a motor directly from the app so you can test it without robot code (robot must be enabled, with safety interlocks).
- **Plotting**: graphs signals in real time to help with PID and feedforward tuning.
- **Signal logging**: retrieves and converts the hoot log files that Phoenix 6 records.
- **Swerve project generator**: creates a starting CTRE swerve drivetrain project from your module configuration.
- **CANivore management**: sets up and updates CANivore USB-to-CAN adapters.

### Connecting

It connects to the robot over USB, Wi-Fi or Ethernet through a diagnostic server that runs on the roboRIO or a coprocessor. It also connects directly to a CANivore plugged into your computer.

It's the CTRE counterpart to REV Hardware Client, which does the same job for REV devices such as SPARK MAX.

## CANivore

CANivore is a USB-to-CAN FD adapter from CTRE. It adds an extra, faster CAN bus to an FRC robot alongside the roboRIO's built-in one.

### Why teams use it

- **Speed**: runs CAN FD (Flexible Data-rate) at up to 5 Mbps with larger frames, vs. classic CAN at 1 Mbps on the roboRIO bus. Much more bandwidth.
- **Less bus load**: swerve drives (8 motors + 4 CANcoders + a Pigeon gyro) can push the roboRIO bus to its limit, so teams often move those devices to a CANivore.
- **Faster, synchronized data**: Phoenix 6 devices on a CANivore can send status signals at high rates. Phoenix Pro features such as time-synced signals and fused CANcoder feedback work best on CAN FD. Helps odometry and control.
- **Isolation**: a wiring fault on one bus doesn't take down the other.

### How it works

- Plugs into a USB port on the roboRIO, or on a coprocessor such as a Raspberry Pi or Orange Pi.
- Shows up as a named CAN bus (e.g. `"canivore"` or a custom name). In code, pass the bus name when creating a device: `new TalonFX(1, "canivore")`.
- Only CTRE Phoenix 6 devices get the CAN FD benefits: Talon FX, CANcoder, Pigeon 2, CANdle. Non-CTRE devices (REV SPARK MAX, pneumatics hubs) usually stay on the roboRIO bus.
- Configured and firmware-updated with Tuner X. Can also plug directly into a laptop to test devices without a roboRIO.

### Hardware notes

- It's a physical device: a small box (about matchbox-sized) with a USB-C port on one end and CAN wiring terminals on the other. You mount it on the robot, run USB to the roboRIO or a coprocessor, and wire CAN devices to it. Costs about $300 from CTRE.
- The `"canivore"` bus name in code and in Tuner X refers to this physical adapter, not a separate software product.
- Built-in termination resistors that can be switched on or off, plus status LEDs.
- Small and light; powered from USB.

## roboRIO

The roboRIO is the main robot controller for FRC: the robot's brain. National Instruments (NI) makes it, and every FRC robot is required to have one. Your robot code runs on it.

### What's inside

- **Xilinx Zynq chip**: a dual-core ARM Cortex-A9 CPU at 667 MHz combined with an FPGA. The FPGA handles timing-critical I/O such as reading encoders and generating PWM signals, so the CPU doesn't have to.
- **RAM**: 256 MB on the original, 512 MB on the roboRIO 2.0.
- **OS**: NI Linux Real-Time (Linux with real-time patches). Java, C++ or Python robot code runs on it as a normal process.
- **Storage**: built-in flash on the original. The roboRIO 2.0 boots from a microSD card, so you can reimage it by swapping cards.

### Ports and I/O

- PWM outputs (10), digital I/O (10), analog inputs (4), relay outputs (4).
- CAN bus for motor controllers, the power distribution hub, the pneumatics hub and sensors.
- I2C, SPI and RS-232 serial.
- MXP expansion port for extra I/O and add-on boards (e.g. navX gyros).
- USB host ports (cameras, CANivore) and a USB device port (connecting a laptop).
- Ethernet, which connects to the robot radio.

### Its role on the robot

- Talks to the Driver Station through the radio: receives joystick input and enable/disable commands, sends back status.
- Enforces safety: outputs are disabled unless the Driver Station or the field enables the robot.
- Gets 12 V power from the Power Distribution Panel or Hub (PDP/PDH).
- Imaged and configured with the roboRIO Imaging Tool; code is deployed from VS Code with WPILib.

### In this repo (AOS)

AOS-based robots run their control loops on the roboRIO, using WPILib's HAL only for low-level hardware access, and often offload vision and other heavy processing to coprocessors. (Not yet confirmed against this repo's code.)

## Differential drivetrain

A differential drivetrain (also called tank or skid-steer drive) has two independently driven sides, left and right. It steers by running the two sides at different speeds; no wheel turns to point in a new direction.

### How it moves

| Left side | Right side | Result |
|---|---|---|
| Same speed forward | Same speed forward | Straight ahead |
| Faster | Slower | Curves right |
| Forward | Backward, same speed | Spins in place (to the right) |

Kinematics, with `w` = track width (distance between left and right wheels):

```
v     = (v_left + v_right) / 2      forward speed
omega = (v_right - v_left) / w      turn rate (counterclockwise positive)
```

Inverse (command → wheel speeds):

```
v_left  = v - omega * w / 2
v_right = v + omega * w / 2
```

### Hardware

- Usually 2 or 3 motors per side, ganged into one gearbox, driving 4–8 wheels per side on chains or belts.
- An encoder on each side, plus a gyro for heading.
- Variants: West Coast Drive (dropped centre wheel, so it rocks onto fewer wheels and turns more easily), the FRC kit-of-parts chassis, tank treads.

### Compared with swerve (what this robot uses)

| | Differential | Swerve |
|---|---|---|
| Directions it can move | Forward/back + turning only; can't strafe (nonholonomic) | Any direction while facing any heading (holonomic) |
| Motors | 4–6 | 8 (drive + steer per module) + 4 absolute encoders |
| Cost, weight, complexity | Low | High |
| Pushing power | High; all motors push the same way | Lower per motor |
| Control | Simple; two velocities | Per-module angle and speed; needs good odometry |
| Odometry | Good, but wheel scrub when turning adds error | Good if modules are well calibrated |
| Path following | Needs special controllers (e.g. Ramsete, LTV) since it can't move sideways | Simple; correct x, y and heading independently |

Most top FRC teams switched to swerve once commercial modules (SDS, WCP, REV) became common, mainly for strafing and turning while driving. Differential drive is still common for its cost, simplicity and pushing power.

### In this repo

- `offseason2026` is swerve (CTRE `TunerSwerveDrivetrain`, four modules), not differential.
- The vendored AOS contains 971's full differential drivetrain in `third_party/aos/frc/control_loops/drivetrain/`: state-space controller (`polydrivetrain`), EKF localizer (`hybrid_ekf.h`), spline path following (`distance_spline`, `splinedrivetrain`), gear shifting (`gear.h`). Not used by this team; the place to start for a tank-drive robot.
- WPILib equivalents: `DifferentialDrive` (arcade/tank/curvature driving), `DifferentialDriveKinematics`, `DifferentialDriveOdometry`.

## Bringing up the robot (offseason2026 on the roboRIO)

Worked out from `offseason2026/BUILD.bazel`, `third_party/aos/frc/config/setup_roborio.sh` and the AOS downloader. Not yet run against a real robot. Run commands from the repo root, with the laptop on the robot's network.

### 1. Flash the roboRIO (once per RIO)

- Use the NI/WPILib roboRIO Imaging Tool to install the 2026 image and set team number 1868. The 2026 image already includes the NI runtime libraries, so the deploy doesn't copy them.
- The RIO is then at `roboRIO-1868-frc.local` (or `10.18.68.2` on a static setup).

### 2. One-time AOS setup on a freshly flashed RIO

```
./tools/bazel run @aos//frc/config:setup_roborio -- roboRIO-1868-frc.local
```

Run before the first deploy. It:
- creates `/home/admin/bin` and points the log directory at the USB stick (`/media/sda1`)
- adds `/home/admin/bin` to `PATH`
- changes memory settings (`vm.overcommit_memory=0`, `vm.min_free_kbytes`)
- stops the default FRC robot program
- installs a `robotCommand` that launches `/home/admin/bin/starter.sh` at boot

Needs a USB stick in the RIO for logs.

### 3. Build and deploy

```
./tools/bazel run --config=roborio //offseason2026:download_stripped -- roboRIO-1868-frc.local
```

Cross-compiles for the RIO and rsyncs everything into `/home/admin/bin`:
- `starterd` (the AOS process manager)
- `wpilib_interface`
- `drivetrain`
- `aos_timing_report_streamer`
- `aos_config.json`
- the stripped WPILib, CTRE and OpenCV `.so` files

The first time, the deploy script installs `jq` and `rsync` on the RIO from NI's package feed. If the sandboxed ssh/rsync crashes on your machine, add `--@aos//frc/downloader:rsync_mode=host`.

Use `download_stripped`, not `download`: the unstripped WPILib libraries are huge (`libwpimath.so` alone is about 435 MB).

### 4. Start it

- Reboot the RIO or restart robot code from the Driver Station. `robotCommand` → `starter.sh` → `starterd` launches the applications in the config: `wpilib_interface` and `drivetrain` (defined in `offseason2026/drivetrain/drivetrain.json`).
- To check: SSH in as `admin` and run `aos_starter status`. To see message traffic, use `aos_dump /drivetrain` and similar.

### 5. Before enabling

- In Tuner X, check every CTRE device shows up with up-to-date firmware and the right CAN IDs. If IDs change, the generated `TunerConstants` must match.
- Connect the Driver Station, confirm comms and robot code, then enable with the robot on blocks first.

## Dependency mirror

Three layers: generic downloads go through a URL rewrite in Bazel's downloader, Bazel modules come from a mirrored copy of the Bazel Central Registry (BCR), and Python wheels come from our own pip index. All three are files under `/mnt/mfs/bazel_deps_mirror/` on the spacecookies server, served by a static nginx at `https://mirror.spacecookies.dev`. Nothing fetches or caches on demand: a file is only there if someone put it there with one of the `tools/mirror_*` scripts.

### 1. URL rewriting (http_archive, http_file, toolchains, crates, ...)

`.bazelrc` sets:

```
common --downloader_config=tools/dependency_rewrite
```

Every Bazel download is checked against that file:

- `rewrite github.com/(.*) mirror.spacecookies.dev/github.com/$1`, plus ~30 similar rules (CTRE maven, WPILib's `frcmaven.wpi.edu`, `static.rust-lang.org`, `crates.io`, `files.pythonhosted.org`, `nodejs.org`, `mirror.bazel.build`, ...). Upstream host + path becomes the directory path on the mirror, so `https://github.com/foo/bar/archive/v1.tar.gz` is served from `/mnt/mfs/bazel_deps_mirror/github.com/foo/bar/archive/v1.tar.gz`.
- `block *` blocks every host not explicitly allowed, so a missing mirror entry fails loudly instead of quietly downloading from the internet.
- A short `allow` list lets a few hosts through directly: `mirror.spacecookies.dev`, `realtimeroboticsgroup.org`, `pypi.org`, `go.dev`, `armkeil.blob.core.windows.net`. (go.dev and pypi.org stay allowed until the Go toolchain and the "simple" Python API are fixed.)

Bazel still checks each artifact's `sha256` from `MODULE.bazel` or the lockfiles, so the mirror can't silently serve a different file.

To add a file: `tools/mirror_tarball.sh <upstream-url>`. It strips the `https://`, creates the matching directory under `/mnt/mfs/bazel_deps_mirror/`, and `curl`s the file into place after a y/N prompt.

### 2. Bazel module registry (bzlmod)

`.bazelrc` lists three registries, searched in order:

```
common --registry=file://%workspace%/registry                  # our own modules/overrides
common --registry=file://%workspace%/third_party/aos/registry  # AOS's
common --registry=https://mirror.spacecookies.dev/bcr.bazel.build
```

The third is a partial copy of `bcr.bazel.build`. To add a module: `tools/mirror_registry_entry.sh <module> <version>`. It downloads the module's `metadata.json` plus that version's `MODULE.bazel`, `source.json`, patches, overlay files and `presubmit.yml` if present. It uses `curl -f` and writes through a temp file, so an HTTP error page can't end up saved as JSON.

That script only mirrors registry metadata. The module's source archive (the `url` in `source.json`, usually on GitHub) goes through the layer 1 rewrite rules, so a new module usually also needs `mirror_tarball.sh` on that URL, or the build hits `block *`. (Worked out from reading the scripts, not tested.)

### 3. Python wheels

- AOS's `pip.parse` downloads wheels with Bazel's downloader (`experimental_index_url`). The index is set by `common --repo_env=AOS_PIP_INDEX_URL=https://mirror.spacecookies.dev/pip/simple`.
- That's a PEP 503 "simple" index we own. `tools/mirror_pip_wheels.py` builds it from the lockfiles (`third_party/aos/tools/python/requirements.lock.txt` and `aos_requirements.lock.txt`): cp310/cp313 x86_64 and pure-Python wheels from RTR's wheelhouse, falling back to PyPI, with a `#sha256=` on every index link.
- Re-run it whenever a lockfile changes. A non-empty `MISSING` list means no wheel exists upstream for one of our platforms. `--platform macos-arm64` and `--build-missing` cover macOS (see the README).

### Remote builds

With `--config=remote`, `--experimental_remote_downloader=grpcs://engflow.spacecookies.dev` sends downloads through EngFlow, which fetches the same rewritten mirror URLs and caches them in the remote store. `--experimental_remote_downloader_local_fallback` means your machine downloads the file itself if EngFlow can't.

### Discrepancies found

- The README says CI sets `AOS_PIP_EXTRA_INDEX_URL` to our mirror, but `.bazelrc`'s `running_in_ci` config points both `AOS_PIP_INDEX_URL` and `AOS_PIP_EXTRA_INDEX_URL` at `realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple` (with `--trusted-host`). So CI currently pulls wheels from RTR, not our mirror. Either the README or the CI config is out of date.
- `tools/dependency_rewrite` begins with "Use tools/rehost.sh to rehost easily", but there's no `tools/rehost.sh`. The scripts are `mirror_tarball.sh` and the other two.
- `tools/dependency_rewrite_permissive` is a transition leftover (LLVM and Foxglove → RTR, ARM toolchain → Azure). `.bazelrc` doesn't reference it, so it's only used if passed manually.
- `mirror_tarball.sh`'s header says `mirror_registry_entry.sh` (copied from that script). Harmless.

### When a build fails on a new dependency

1. The error names a blocked or 404 URL on `mirror.spacecookies.dev/...`.
2. On the spacecookies server, run `tools/mirror_tarball.sh <original-upstream-url>`, or `mirror_registry_entry.sh` for a new BCR module and then `mirror_tarball.sh` for its source archive.
3. Re-run the build. The `sha256` check confirms the file is the right one.

## IMU and wheel sensors (offseason2026)

Yes, the robot has both, through CTRE's swerve library rather than separate code of its own, and both end up in the `/drivetrain` `Position` message. (From reading the code and the Tuner X project file; not checked on the robot.)

### Hardware

From `offseason2026/drivetrain/generated/TunerConstants.h` and `tuner-project.json`:

- **IMU**: one Pigeon 2.0, ID 0, on the CAN bus named `"CANivore"`. `pigeonConfigs` is `std::nullopt`, so the code applies no Pigeon settings and uses whatever is stored on the device.
- **Wheels**: four swerve modules, each with a Talon FX drive motor, a Talon FX steer motor and a CANcoder, all on the CANivore.
  - Front left: drive 0, steer 1, CANcoder 0
  - Front right: drive 6, steer 7, CANcoder 3
  - Back left: drive 2, steer 3, CANcoder 1
  - Back right: drive 4, steer 5, CANcoder 2
- Steering uses `FusedCANcoder` feedback, a Phoenix Pro feature, so the devices need Pro licenses.

### How the data reaches AOS

`wpilib_interface.cc` creates the `TunerSwerveDrivetrain`. CTRE's own odometry thread combines the Pigeon yaw with the module encoders. Every update, `RegisterTelemetry` calls `SwervePositionSender::Send`, which publishes `offseason2026.drivetrain.Position` on `/drivetrain`:

| Field | Source | Use |
|---|---|---|
| `x`, `y`, `theta` | CTRE odometry (wheels + gyro) | Pose on the field |
| `raw_heading` | Pigeon yaw, unaffected by vision or odometry resets | Turning (see `control_example.md`) |
| `vx`, `vy`, `omega` | Measured robot-relative velocity | Feedback, checking if settled |
| `module_states` | Per-wheel speed (m/s) and angle | Wheel-level debugging |
| `module_targets` | What CTRE is commanding each module to | Commanded vs. actual |
| `module_positions` | Per-wheel distance driven (m) and angle | Your own odometry or distance driven |
| `timestamp`, `odometry_period`, `successful_daqs`/`failed_daqs` | CTRE | CAN and odometry health |

`drivetrain` copies this into its `Status` message. Read it with `MakeFetcher<Position>("/drivetrain")` or a watcher. On the robot, `aos_dump /drivetrain offseason2026.drivetrain.Position` shows it live.

### What's missing

- **Only yaw comes through.** Pitch, roll, angular rates and acceleration from the Pigeon aren't published (CTRE's `SwerveDriveState` doesn't include them). You'd need to read `drivetrain.GetPigeon2()` signals and add fields to `Position` or a new message. Useful for tip detection, balancing and collision detection.
- **Nothing on the team side consumes this data yet**, apart from passing it through to `Status`. No auto, heading controller or localizer in `offseason2026` reads it. `2026/vision/aimer` uses a different message, `frc::controls::LocalizerOutput` from AOS, and nothing in `offseason2026` produces that message.
- AOS has its own IMU code (`third_party/aos/frc/imu`, `imu_reader`, `imu_fdcan`), but that's for 971's custom IMU boards. This team's code doesn't use it.

### One thing to check

Confirm the CAN IDs in Tuner X match the real robot before relying on them. Several devices share numbers (Pigeon 0, CANcoder 0, drive motor 0), which is fine in CTRE because IDs only have to be unique per device type. But if a device was re-IDed after the constants were generated, the mismatch only shows up at runtime. The CTRE log goes to `./logs/example.hoot`, relative to the process's working directory on the RIO.

## Multi-process vs. multi-thread

Both, but mainly multi-process. AOS splits the robot into separate processes that talk over shared memory. A few processes, mainly the one that talks to hardware, also run several threads inside them.

### Multi-process: the AOS design

- Each application in the config (`robot.json` / `drivetrain.json`, `single_node_camera_config.json`) is its own Linux process.
- `starterd` launches them, restarts them if they crash, and reports their status (`aos_starter status`).
- Processes don't call each other. They publish and subscribe to FlatBuffer messages on named channels in shared memory (`ShmEventLoop`), e.g. `/drivetrain` `Goal`, `Position`, `Output`.
- Each process usually runs one event loop on one thread, so control code like `Drivetrain::RunIteration` needs no locks.
- One process crashing doesn't take down the others, and any one can be logged, replayed or restarted on its own.

On the roboRIO (`offseason2026`), `drivetrain.json` lists:

| Process | Role |
|---|---|
| `starterd` | Process manager |
| `wpilib_interface` | The only process that touches hardware |
| `drivetrain` | Control loop: `Goal` + `Position` → `Output` |

On the Orin (`2026/vision`), about 25 processes: per camera `camera_readerN`, `turbojpeg_decoderN`, `apriltag_detectorN` (×4), then `localizer_main`, `aimer`, `aimer_forwarder`, loggers, monitors, etc.

### Multi-threaded inside `wpilib_interface`

Deliberately multi-threaded. Per the comment in `wpilib_interface.cc`, the WPILib HAL only allows one process at a time (`HAL_Initialize` kills whichever process holds `/var/lock/frc.pid`), so everything needing WPILib or Phoenix lives in this process.

| Thread | What it does |
|---|---|
| CTRE odometry thread (created by Phoenix) | Runs the telemetry callback; sends `RobotState` and `Position` through its own `ShmEventLoop` (`swerve_odometry`), which only sends and is never run |
| `joystick_thread` | Runs `JoystickSenderHal`; sends `JoystickState` whenever a Driver Station packet arrives |
| Main thread | Runs `DrivetrainWriter`; passes `Output` to CTRE's swerve |

Each thread creates, runs and destroys its own event loop, because AOS senders and watchers must be used by the thread that created them. `EndCompetition` uses a mutex-protected list to tell them all to exit. WPILib, NI and Phoenix also start internal threads of their own, which is why `die_on_malloc` is off: their real-time threads allocate memory.

### In short

A set of single-threaded processes linked by shared-memory message channels. The only real exception is `wpilib_interface`, multi-threaded because the HAL forces all hardware access into one process.

Gap: `super_loop/joysticks.cc` (joystick → `Goal`) isn't listed as an application in `drivetrain.json`. `wpilib_interface` sends raw `JoystickState`, but nothing deployed turns it into drivetrain `Goal`s yet.

## AOS vs. ROS 2

Both use the same basic idea: separate processes that publish and subscribe to typed messages on named channels. AOS gives up flexibility to get predictable, repeatable real-time behaviour; ROS 2 gives up some of that predictability for flexibility and a huge ecosystem. (AOS side from this repo's code; ROS 2 side from general knowledge, since there's no ROS 2 code here.)

### Concept mapping

| AOS (this repo) | ROS 2 |
|---|---|
| Application (one process each, launched by `starterd`) | Node. Several nodes can share one process ("composition"). |
| Channel (name + FlatBuffer type), e.g. `/drivetrain` `Goal` | Topic + message type, e.g. `/cmd_vel` `geometry_msgs/Twist` |
| `Sender` / `Watcher` | Publisher / subscription callback |
| `Fetcher` (read the latest message whenever you want) | No direct equivalent; usually keep-last-1 QoS plus caching it yourself |
| `EventLoop`, `AddTimer`, `AddPhasedLoop` | Executor, timers |
| `aos_config` JSON: every channel's type, max size, max rate, source node, logging | No central config. Topics appear at runtime; QoS is set per publisher/subscriber |
| `maps` (rename `/camera` → `/camera3`) | Topic remapping |
| `starterd` + `applications` (auto-restart) | Launch files + `respawn`, lifecycle nodes |
| AOS logger + `LogReader` replay | rosbag2 (MCAP) |
| `aos_dump` | `ros2 topic echo` |
| Constants channels (`vision_constants_sender`) | Parameters |
| `message_bridge` between nodes, listed in the config | DDS (or Zenoh) discovery across the network, automatic |
| `SimulatedEventLoopFactory` (simulated time, many nodes in one test) | `/clock` sim time, `launch_testing`, Gazebo |
| No general services or actions (only starter RPC) | Services, actions |

### Key differences

#### 1. Transport

- **AOS**: lock-free queues in shared memory, allocated up front at a fixed size per channel. With `read_method: PIN`, readers see the sender's buffer directly with no copy. Messages are FlatBuffers, read in place with no decode step. Messages leave the machine only through `message_bridge`, and only for channels the config lists.
- **ROS 2**: by default goes through DDS (RTPS over UDP) and serializes every message. Avoiding copies is possible (composition, loaned messages, shared-memory transports, NVIDIA's NITROS) but opt-in and middleware-dependent.
- Why it matters here: the Orin moves 4 cameras × ~2 MB grayscale frames (1600×1304) at up to 125 Hz, roughly 1 GB/s, through `/cameraN/gray`. AOS gets zero-copy by default; ROS 2 would need composed nodes or GPU-specific transports to keep up.

#### 2. Static vs. dynamic system shape

- **AOS**: the whole system is declared up front (each channel's type, size limit, max rate, sending node, whether it's logged). `aos_config` checks it at build time; going over a channel's rate limit is an error. You know exactly what the system does before it runs, but adding a camera means editing the config (and, here, some C++).
- **ROS 2**: nodes and topics appear and disappear at runtime through discovery. Easy to plug in a new node, but QoS mismatches, discovery problems and unexpected traffic only show up while running.

#### 3. Real time

- **AOS**: built for it. One thread per event loop, no allocation after startup (`die_on_malloc`), real-time priorities (`aimer` at priority 2), CPU and IRQ pinning (`irq_affinity`), fixed-size queues. `wpilib_interface.cc` only turns `die_on_malloc` off because WPILib, NI and CTRE code allocates.
- **ROS 2**: possible but takes work: real-time kernel, custom allocators, careful executor choice, middleware tuning. Default executors and DDS layers aren't predictable in timing.

#### 4. Testing and replay

- **AOS**: probably its biggest advantage. `SimulatedEventLoopFactory` runs a whole multi-process, multi-node system in one test with simulated time (`aimer_test` does this). Log replay is deterministic: exact message order and timestamps, across nodes too, with clock offsets estimated. Re-run a changed `localizer_main` against a match log and get repeatable results (`localizer_replay.sh`). Extrinsics calibration works the same way.
- **ROS 2**: rosbag2 replays messages, but callback order and timing under load can vary between runs. Whole-system tests usually run as real separate processes.

#### 5. Ecosystem

- **ROS 2**: far bigger: tf2 (coordinate frames), robot_localization, Nav2, MoveIt, ros2_control, image_pipeline, apriltag_ros, Isaac ROS (GPU AprilTag), camera drivers, RViz, large community and docs.
- **AOS**: small, from 971/RTR, FRC-focused. You get the FRC-specific pieces this repo uses (CUDA AprilTag detector, swerve localizer, shoot-while-moving aiming, roboRIO deploy) ready-made, but little else, and docs are thin. Things like the 4-camera limit are hard-coded.

#### 6. Platform and build

- **AOS**: Bazel monorepo with fixed toolchains, cross-compiling to the roboRIO (ARMv7, NI Linux RT) and the Orin (arm64 + CUDA). One `bazel run …:download_stripped` deploys everything.
- **ROS 2**: colcon/ament on Ubuntu with apt packages. Works very well on the Orin (JetPack is Ubuntu), but the roboRIO isn't a practical target (old NI Linux RT, ARMv7, limited RAM). FRC teams using ROS 2 typically keep WPILib on the roboRIO and run ROS 2 only on the coprocessor, connected by NetworkTables or UDP. Similar to this repo's RIO/Orin split (`aimer_forwarder` sends UDP to the RIO), but the two halves would run different frameworks; here AOS runs on both.

#### 7. Learning curve

- ROS 2 knowledge is widespread and transfers to university and industry robotics.
- AOS is niche; students mostly learn it from the code and from 971/RTR.

### For this repo specifically

| Concern | Winner | Why |
|---|---|---|
| Hard real-time drivetrain on the roboRIO | AOS | ROS 2 doesn't really run on the roboRIO |
| 4 high-rate cameras on the Orin | AOS (out of the box) | Zero-copy shared memory by default; ROS 2 needs composition, NITROS or Isaac ROS |
| Debugging a match afterwards | AOS | Deterministic replay of the whole system |
| Adding new sensors or algorithms quickly | ROS 2 | Drivers and packages exist; dynamic discovery |
| Navigation, path planning, frames | ROS 2 | Nav2, tf2 |
| Onboarding students | ROS 2 | Docs, tutorials, community |
| One toolchain for RIO + Orin | AOS | Same framework, build and deploy on both |

Combined option some teams use: WPILib or AOS on the roboRIO for control, ROS 2 on the coprocessor for vision and planning, connected by NetworkTables. Costs: no single log or deterministic replay across both machines, and two build systems to maintain.

Assessment: for an FRC robot relying on fast, real-time swerve control and multi-camera localization, AOS is the better technical fit, at the cost of a smaller ecosystem and steeper learning curve. ROS 2 pays off mainly for off-the-shelf navigation or perception packages, or if students learning widely used tools is the priority.

## Where the main functions are

There isn't one main function that runs the robot. Each AOS application has its own `main()`, and each starts its own event loop with `event_loop.Run()`. (From reading the code; not run.)

### On the roboRIO (offseason2026)

#### Control loop: `offseason2026/drivetrain/drivetrain_main.cc`

```cpp
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  offseason2026::drivetrain::Drivetrain drivetrain(&event_loop);

  event_loop.Run();   // <-- the loop
  return 0;
}
```

Builds into `//offseason2026/drivetrain:drivetrain`, which `starterd` launches as the `drivetrain` application.

The loop doesn't run on a timer. The `Drivetrain` constructor goes through `frc::controls::ControlLoop` (`third_party/aos/frc/control_loops/control_loop.h`), which sets up:
- fetchers for `Goal` on `/drivetrain`, and `RobotState` and `JoystickState` on `/frc`
- a watcher on `/drivetrain` `Position` (line 91)

Each time CTRE's odometry thread publishes a `Position`, the watcher calls `IteratePosition()` → `Drivetrain::RunIteration(goal, position, output, status)` in `drivetrain.cc`. The loop runs at the odometry rate.

#### Hardware process: `offseason2026/wpilib_interface.cc` (line 126)

```cpp
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  absl::SetFlag(&FLAGS_die_on_malloc, false);
  return frc::StartRobot<offseason2026::Robot>();
}
```

`frc::StartRobot` is WPILib's entry point. It calls `Robot::StartCompetition()`, which:
- creates the CTRE drivetrain and registers the telemetry callback (on CTRE's odometry thread)
- starts `joystick_thread`, whose own `event_loop.Run()` runs `JoystickSenderHal`
- runs `DrivetrainWriter` on the main thread with `event_loop.Run()`

### On the Orin (2026/vision)

- `2026/vision/aimer.cc`: `main()` at line 26.
- `2026/vision/aimer_forwarder.cc`: `main()` at line 147.
- The rest (`camera_reader`, `apriltag_detector`, `localizer_main`, ...) have their `main()`s in `third_party/aos/frc/vision/...`.

### What's not a robot program

- `offseason2026/super_loop/main.cpp` is a gtest runner (`RUN_ALL_TESTS()`).
- `super_loop` and `joysticks` are only `cc_library` targets. Nothing has a `main()` that creates `JoystickInterface` or the superstructure loop, so neither runs on the robot yet. That's why driver input never reaches the drivetrain. Fix: add a `cc_binary` with a `main()` like `drivetrain_main.cc`, plus a matching entry in the config's `applications`.

## Code base issues

Review of the team's own code (`offseason2026`, `2026/vision`, `workshops2026`, `tools`), not the vendored AOS. Snapshot as of commit `ce7bc037`.

### Build and test results

| Command | Result |
|---|---|
| `bazel test //offseason2026/...` (host) | 3 of 3 pass |
| `bazel test //2026/vision:aimer_test` | Pass |
| `bazel build --config=roborio //offseason2026:download_stripped` | Builds (about 13 minutes) |
| `bazel build //2026/vision:constants.json` | **Fails** (issue 3) |

One of the three passing offseason tests, `super_test`, actually runs 0 tests (issue 6).

### Serious: the robot won't work as intended

1. **Nothing turns driver input into drive commands.** `JoystickInterface` is defined only inside `super_loop/joysticks.cc`, part of a library. No program creates it, and `drivetrain.json` lists only `wpilib_interface` and `drivetrain`. The superstructure loop isn't deployed either, and its `/superstructure` channels aren't in the deployed config. Enabling the robot does nothing.
2. **`joysticks.cc` bugs that will show up once it runs:**
   - Never calls `set_mode(...)`, so every goal is `kIdle`.
   - `kLeftStickX`, `kLeftStickY`, `kRightStick` are all `{0, 2}`. AOS axes are 0-based, so all three read the left trigger, which also controls the intake. On an Xbox controller: left stick = axes 0 and 1, right stick X = axis 4.
   - Stick X drives `vx` and stick Y drives `vy`, no sign flips. Usual mapping is `vx = -leftY`, `vy = -leftX`. Rotation probably needs negating too.
   - `kMaxAngularSpeed = 1.25` is rad/s (`Goal.omega` is rad/s), about 72°/s, slow for swerve. May have been meant as rotations per second.
3. **Vision deploy can't build on a clean cache.** `constants.json` goes through jinja2, which needs `markupsafe==2.1.5` for the Python 3.13 host interpreter. The mirror only has cp310 wheels, and no cp313 Linux wheel or sdist exists, so the fetch fails. Blocks `//2026/vision:download_stripped`. The README says MarkupSafe is only a macOS gap; it's a Linux gap too. May still work on machines whose Bazel cache already has it.
4. **Calibration data crashes `localizer_main` on some robots:**
   - `stick` has `camera_number` 0 twice and no 1.
   - `test` has 0, 2, 3, 3.
   - `stick` and `testboard` files mostly have no `fixed_extrinsics`.
   - `glued/calibration_orin-1868-3_cam-26-11_...json` says `camera_id: "25-11"`.
5. **The roboRIO code doesn't feed the Orin.** `localizer_main` and `aimer` need RIO pose and speed data over UDP port 4647 (republished as `/drivetrain` `Pose2d` and `ChassisSpeeds`), plus AdvantageKit NetworkTables topics. `aimer_forwarder` sends to port 4649. `wpilib_interface.cc` sends nothing to the Orin and listens for nothing from it, so vision localization and aiming don't work with this robot code.

### Medium

6. **`super_test` gives a false pass.** All of `super_test.cc` is commented out, but the target passes with 0 tests. `super_loop/main.cpp` isn't used by any target. All `IntakeConstants` pivot positions are `0.0 // TODO`.
7. **`aimer` can crash at startup.** Its `ConstantsFetcher` fails a check ("Constants information must be available at startup") if it starts before `field_map_constants_sender` has published. `aimer.cc` doesn't call `WaitForConstants` the way `localizer_main.cc` does. `starterd` restarts it, so it should recover, but may crash at least once per boot.
8. **`aimer` speed table unfinished.** In `aimer.h`, `< 3.5` and the `else` case both give `shot_velocity = 3.5`.

### Low

9. **Fragile config imports.** `drivetrain.json`, `super_loop/robot.json` and `cookie.json` import `../../external/aos+/...`, which depends on Bazel's internal name for the AOS module.
10. **CTRE log path.** CTRE logs to `./logs/example.hoot` relative to the working directory. `setup_roborio.sh` creates the USB log link at `/home/admin/logs`, not under `bin/`. Check where the hoot file actually ends up.
11. **Shared UDP port.** `aimer_forwarder` and `network_tables_swerve_client` (game pieces) both default to roboRIO port 4649. Object detection is off, so no conflict yet.
12. **Doc problems:** README and CI pip index disagree; `tools/rehost.sh` is referenced but doesn't exist; `mirror_tarball.sh` has the wrong header. `ARCHITECTURE.md` and `REPO_OVERVIEW.md` say "4 Orins per robot", but there's one Orin with 4 cameras.

Not counted: `workshops2026/cookie_loop` (deliberately commented-out student exercise). Not reviewed: `experimental/apriltag_detect/`.

**Quickest path to a drivable robot:** issues 1 and 2 (add a joystick application, fix the axis mapping).


## [Practical Basel](https://www.stevenengelhardt.com/2023/03/08/practical-bazel-using-clang-12-c-toolchain-on-ubuntu-20.04/)

## [Basel Tutolrial](https://bazel.build/tutorials/ccp-toolchain-config)