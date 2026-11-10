# Control Example: Turning the Robot 90° to the Right

A right turn means a **negative `omega`**. In `offseason2026/drivetrain/drivetrain_goal.fbs`, `omega` is in rad/s with counterclockwise as positive, so clockwise (a right turn, viewed from above) is negative.

There are three ways to do the turn, from crudest to best.

## Table of Contents

- [1. Driver turns it with the stick](#1-driver-turns-it-with-the-stick)
- [2. Timed turn (open loop)](#2-timed-turn-open-loop)
- [3. Closed-loop turn on the gyro heading (recommended)](#3-closed-loop-turn-on-the-gyro-heading-recommended)
  - [Notes](#notes)
  - [Alternative: facing-angle mode in the drivetrain](#alternative-facing-angle-mode-in-the-drivetrain)

## 1. Driver turns it with the stick

The driver pushes the rotation stick until the robot has turned 90°. `offseason2026/super_loop/joysticks.cc` maps the right stick to `omega` at up to `kMaxAngularSpeed = 1.25` rad/s.

As written, that path won't move the robot yet (from reading the code, not from running it):

- It never calls `set_mode(...)`, so every goal goes out with the default `kIdle` and the drivetrain sends idle requests. It needs `set_mode(Mode::kFieldCentric)` or `kRobotCentric`.
- `kLeftStickX`, `kLeftStickY`, `kRightStick` and `kLeftTrigger` are all `{0, 2}`, with an "add correct indices" TODO. The real axis numbers need to go in.
- The sign may be backwards. A positive axis value (stick pushed right, in WPILib's usual convention) gives positive `omega`, which turns the robot **left**. Most teams negate it. Check this on blocks.
- `offseason2026/drivetrain/drivetrain.json` only lists `wpilib_interface` and `drivetrain` as applications, so it isn't clear the joystick code is deployed by `//offseason2026:download_stripped` at all.

## 2. Timed turn (open loop)

Send `omega = -ω` for `(π/2)/ω` seconds, then send 0. At 1.25 rad/s that's about 1.26 s.

Simple, but wheel slip, battery voltage and acceleration all make it miss, often by 10° or more.

## 3. Closed-loop turn on the gyro heading (recommended)

`Position` already carries `raw_heading` from the Pigeon gyro, in radians with counterclockwise positive. Record the heading when the turn starts, set the target to that heading minus π/2, and drive `omega` from the heading error until it's within tolerance.

Sketch as a small AOS application (**not built or tested**):

```cpp
#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

#include "aos/events/event_loop.h"
#include "offseason2026/drivetrain/drivetrain_goal_static.h"
#include "offseason2026/drivetrain/drivetrain_position_generated.h"

namespace offseason2026::drivetrain {

// Turns the robot 90 degrees clockwise (to the right) in place, using the gyro.
class TurnRight90 {
 public:
  explicit TurnRight90(aos::EventLoop *event_loop)
      : position_fetcher_(event_loop->MakeFetcher<Position>("/drivetrain")),
        goal_sender_(event_loop->MakeSender<GoalStatic>("/drivetrain")) {
    // Run at 100 Hz rather than on every Position (CTRE odometry is much faster)
    // so we stay under the Goal channel's frequency limit.
    event_loop->AddPhasedLoop([this](int) { Iterate(); },
                              std::chrono::milliseconds(10));
  }

  bool done() const { return done_; }

 private:
  static constexpr double kP = 3.0;              // (rad/s) per rad of error
  static constexpr double kMaxOmega = 2.0;       // rad/s
  static constexpr double kTolerance = 0.03;     // rad (~1.7 deg)
  static constexpr double kSettledOmega = 0.05;  // rad/s

  void Iterate() {
    if (done_ || !position_fetcher_.Fetch()) {
      return;
    }
    const Position &position = *position_fetcher_;
    const double heading = position.raw_heading();

    // Counterclockwise is positive, so "right" is minus 90 degrees.
    if (!target_) {
      target_ = heading - std::numbers::pi / 2.0;
    }

    // Shortest signed angle from heading to target, in [-pi, pi].
    const double error =
        std::remainder(*target_ - heading, 2.0 * std::numbers::pi);

    double omega = std::clamp(kP * error, -kMaxOmega, kMaxOmega);
    if (std::abs(error) < kTolerance &&
        std::abs(position.omega()) < kSettledOmega) {
      omega = 0.0;
      done_ = true;
    }

    auto builder = goal_sender_.MakeStaticBuilder();
    builder->set_mode(Mode::kRobotCentric);
    builder->set_vx(0.0);
    builder->set_vy(0.0);
    builder->set_omega(omega);
    builder.CheckOk(builder.Send());
  }

  aos::Fetcher<Position> position_fetcher_;
  aos::Sender<GoalStatic> goal_sender_;
  std::optional<double> target_;
  bool done_ = false;
};

}  // namespace offseason2026::drivetrain
```

### Notes

- It uses `raw_heading` rather than `theta`, because vision updates and odometry resets can make `theta` jump in the middle of a turn.
- `kP` and `kMaxOmega` are guesses. Tune them on blocks, then on carpet. If the robot oscillates, lower `kP`. If it stops short, raise `kP`, or add a minimum `omega` while the error is above tolerance.
- Only one process should send `/drivetrain` `Goal`s at a time. If the joystick app is also sending, the two will fight. Real code would run this as part of an auto routine, or behind a button handled inside the joystick app.

### Alternative: facing-angle mode in the drivetrain

CTRE has `swerve::requests::FieldCentricFacingAngle`, which runs its own heading PID on the motor controller side. To use it, add a `kFieldCentricFacingAngle` mode and a `target_heading` field to `Goal` and `Output`, then handle them in `offseason2026/drivetrain/drivetrain_writer.cc`.
