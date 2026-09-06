# KitBot Differential Drive — Drive/Odometry Project

A standard WPILib KitBot differential drivetrain, extended with odometry, a
physics-based drive simulation, and trajectory generation and following for
autonomous. Robot structure and bindings live in `RobotContainer`; the
drive, sensor, and odometry logic live in `DriveSubsystem`; trajectory
generation lives in `Trajectories`, and path following in the
`FollowTrajectory` command.

## Subsystems

- **DriveSubsystem** — four `SparkMax` drive motors (two leaders, two
  followers), one CTRE CANcoder, one CTRE Pigeon 2 gyro, differential-drive
  odometry, an `LTVUnicycleController` for trajectory following, and a
  `DifferentialDrivetrainSim`-based simulation.
- **Flywheel**, **IntakeClass**, **Loader** — shooter and intake mechanisms,
  bound to the driver controller in `RobotContainer`.

## Controls

All bindings are in `RobotContainer.configureBindings()`, on the driver's
Xbox controller.

| Input | Action |
|---|---|
| Left stick Y / Right stick X | Arcade drive (forward/back, turn) |
| Right trigger (hold) | Run flywheel + feed loader |
| Left trigger (toggle, debounced) | Run intake + loader |
| Right bumper | Increase shooter speed |
| Left bumper | Decrease shooter speed |
| A / B / X / Y | Feedforward-only drive test at 1.0 / 2.0 / -1.0 / -2.0 m/s |

Autonomous currently runs `Autos.Trajectory1Auto(m_driveSubsystem)`, which
resets odometry to a generated trajectory's start pose and drives it end to
end. `Autos.driveDistanceAuto()` and `Autos.driveTurnDriveAuto()` remain
available in `Autos.java` as simpler, non-trajectory routines.

## Sensors and odometry

Only one drivetrain encoder is physically installed — a CANcoder on the
**right** side — alongside the Pigeon 2 gyro. There is no left-side encoder.

`DifferentialDriveOdometry` takes heading directly from the gyro rather than
from the wheel encoders, so the two encoder arguments it wants only need to
supply *distance traveled*, not rotation. `DriveSubsystem` exploits this: it
tracks a running total, `m_virtualLeftDistanceMeters`, reconstructed every
cycle from the real right encoder's delta and the gyro's heading delta:

```java
final double deltaHeadingRadians = currentHeading.minus(m_lastHeading).getRadians();
m_virtualLeftDistanceMeters += deltaRight - kDriveTrackWidthMeters * deltaHeadingRadians;
m_lastHeading = currentHeading;
```

This is the same identity `DifferentialDriveKinematics.toChassisSpeeds()` is
built on (`ω = (right − left) / trackWidth`, rearranged), so it holds exactly
for any combination of translation and rotation — not only the straight and
point-turn cases the earlier flag-based version was limited to.
`m_turningInPlace` and `setTurningInPlace()`, the two-branch version this
replaced, have been removed. The virtual value and the real right-side
reading are both passed to `m_odometry.update()`, which cannot tell the
difference between a real sensor and this reconstruction.

Because the reconstruction leans on `kDriveTrackWidthMeters` matching the
robot's actual geometry, any error in that constant now shows up as odometry
drift specifically while turning — worth rechecking against the physical
robot before trusting autonomous on hardware.

## Trajectory generation and following

`Trajectories.generateTrajectory1()` builds a `Trajectory` with
`TrajectoryGenerator.generateTrajectory()`, from three full `Pose2d`
waypoints — start, one interior point, end — rather than bare `Translation2d`
positions. The interior waypoint's heading is set explicitly rather than
left for the generator to infer, which otherwise produced a noticeably wider
path through that point than intended. The `TrajectoryConfig` includes a
`DifferentialDriveKinematicsConstraint`, so the generator slows the whole
path enough to keep both wheels under the configured max speed through tight
curves, not only the chassis centerline.

`FollowTrajectory` is the command that drives it. Each cycle it samples the
trajectory by elapsed time (`Trajectory.sample(t)`) and hands the result to
`DriveSubsystem.driveToTrajectoryState()`, which runs
`LTVUnicycleController.calculate()` against the current odometry pose and
converts the corrective `ChassisSpeeds` to wheel speeds with
`DifferentialDriveKinematics.toWheelSpeeds()`. Because `sample()` keeps
returning the trajectory's final state once its nominal duration has
elapsed, the command does not stop the instant the clock runs out — it holds
that final state, letting the controller's pose-feedback term close any
residual lag, until the estimated pose is within tolerance or a maximum
settle time passes. Wheel-speed commands remain feedforward-only (see
to-do below), so this settle behavior only compensates for lag at the end of
a run, not during it.

`Autos.Trajectory1Auto()` resets odometry to `trajectory1.getInitialPose()`
before handing off to `FollowTrajectory`, and is what
`RobotContainer.getAutonomousCommand()` currently returns.

**Testing without hardware or the simulator.** Building a `Trajectory` has
no HAL dependency, so `TrajectoryDebugTest` (under `src/test/java`) calls
`Trajectories.generateTrajectory1()` directly and prints every state's time,
pose, velocity, and curvature. This is what caught the interior-waypoint
heading problem above, and separately confirmed the generated path's peak
curvature after adding the kinematics constraint — both before deploying or
opening the simulator GUI.

## Simulation

Simulation is enabled. It uses a `DifferentialDrivetrainSim` physics model
driven by two `SparkMaxSim` instances (one per leader) and writes back through
the real CANcoder's and Pigeon 2's simulation state objects. The estimated
pose (from odometry) and the simulator's ground-truth pose are both plotted
on the same `Field2d` widget, as separate named objects, so the two can be
compared visually. This comparison is what validated the odometry
reconstruction above: overlaying the two traces during a trajectory run
showed them separating on curves before the fix, and staying together
through the whole path afterward.

## Known limitations / To-do

- **[Priority] `resetOdometry(Pose2d)` passes the right encoder's raw
  rotations, not meters, into `resetPosition()`.** The argument order is
  correct, but the value should be `getRightDistanceMeters()`, not
  `m_rightEncoder.getPosition().getValueAsDouble()`. Harmless on a clean
  boot — both readings are `0.0` at that point, which is why
  `Trajectory1Auto()` has tested fine so far — but it will corrupt the pose
  reference the moment this runs with nonzero prior motion: a second
  autonomous run in the same sim session without redeploying, or any teleop
  driving before autonomous.
- **No wheel-level closed loop on `setSpeeds()`.** Feedforward voltage only;
  a mismatch between the characterized `kS`/`kV`/`kA` and reality — including,
  in simulation, that the simulated plant has no static-friction term at all
  — has no local correction and is absorbed entirely by
  `LTVUnicycleController`'s next pose-feedback correction. This showed up as
  extra tracking lag through the sharpest part of a generated path; worth
  revisiting if hardware testing shows the same.
- **Verify the CANcoder's sign convention on hardware before trusting
  autonomous.** In simulation the encoder value is derived from the same
  voltage sign driving the simulated plant, so a real-world sign mismatch
  can't surface there. Confirm by hand once on the robot: push it forward
  and check that `getRightDistanceMeters()` increases.
- **`resetEncoderCommand()` / `resetEncoders()` don't reset odometry state.**
  They zero the physical CANcoder but leave `m_lastRightDistanceMeters` and
  `m_virtualLeftDistanceMeters` at their old values, and never call
  `m_odometry.resetPosition(...)`. Not currently bound to a control, but
  calling it as-is would produce a large, incorrect jump in the pose estimate
  on the next cycle.
- **Install the physical left-side CANcoder** and remove the virtual-encoder
  workaround entirely (flagged in `getAverageDistanceMeters()` as Tier 3,
  Task 3).
- **Remove the unused `m_SimModelField`** — ground truth is now plotted via
  `m_field.getObject("Simulation model")` instead, so the second `Field2d`
  instance is dead code (it's never registered with `SmartDashboard.putData`
  either). Also remove the leftover commented-out
  `m_field.getObject("Odometry")` line in `periodic()`.
- **Consider logging the pose through Epilogue** instead of the three manual
  `SmartDashboard.putNumber` calls in `periodic()`, consistent with how
  `getRightDistanceMeters()` and `getVelocityMetersPerSecond()` are already
  exposed via `@Logged`.
- **Add basic turning commands**: `turnToHeadingDegrees(target)` and
  `turnByAngleDegrees(delta)`, reading the Pigeon 2 directly and terminating
  on heading error (remember continuous input across the ±180° wrap).
  `setTurningInPlace()` no longer exists, so these need nothing special —
  write them against the plain gyro reading and `drive.getPose()`, the same
  as any other autonomous command.
- **Construct autonomous routines and verify their accuracy.** Chain the new
  turning commands with straight-line driving into a multi-segment sequence
  (e.g. drive a square), and check the final estimated pose against where the
  robot actually ended up.
- **Further odometry tests.** A few concrete options, roughly in order of
  effort:
  - In simulation, log `m_virtualLeftDistanceMeters` against
    `m_drivetrainSimulator.getLeftPositionMeters()` (the simulator's true
    left position) and plot the difference — this directly validates the
    reconstruction formula against ground truth rather than just watching
    the robot move plausibly on the field. (A coarser version of this —
    comparing the full estimated pose against `m_drivetrainSimulator.getPose()`
    on the field widget — is what validated the reconstruction above; this
    would isolate the encoder term specifically.)
  - Run a "drive a square" or "drive out and back" routine and measure the
    closing error — how far the final estimated pose is from the expected
    end pose (ideally back at the start) — as a single summary number per
    run.
  - Deliberately mismatch `kTrackWidth` between the reconstruction and the
    simulator's construction to confirm turns (not straight segments) are
    where that error shows up, which is a good way to sanity-check the
    formula's sensitivity before trusting it on the real robot.
  - Once arcade-drive teleop is included in testing, log estimated pose
    drift over a few minutes of mixed driving with the general reconstruction
    now in place — this is the first test that actually exercises it outside
    the flagged straight/point-turn cases the old version was limited to.
  - For a repeatable, dashboard-free check, consider a JUnit test that feeds
    a fixed, known sequence of encoder/gyro readings into `periodic()` and
    asserts the resulting pose against a hand-computed expected value. The
    same headless-JUnit approach already exists on the trajectory side, via
    `TrajectoryDebugTest` above — this would be the odometry equivalent.