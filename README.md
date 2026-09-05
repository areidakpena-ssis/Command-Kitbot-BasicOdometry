# KitBot Differential Drive — Drive/Odometry Project

A standard WPILib KitBot differential drivetrain, extended with odometry and a
physics-based drive simulation. Robot structure and bindings live in
`RobotContainer`; the drive, sensor, and odometry logic live in
`DriveSubsystem`.

## Subsystems

- **DriveSubsystem** — four `SparkMax` drive motors (two leaders, two
  followers), one CTRE CANcoder, one CTRE Pigeon 2 gyro, differential-drive
  odometry, and a `DifferentialDrivetrainSim`-based simulation.
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

Autonomous runs `Autos.driveDistance(m_driveSubsystem)`.

## Sensors and odometry

Only one drivetrain encoder is physically installed — a CANcoder on the
**right** side — alongside the Pigeon 2 gyro. There is no left-side encoder.

`DifferentialDriveOdometry` takes heading directly from the gyro rather than
from the wheel encoders, so the two encoder arguments it wants only need to
supply *distance traveled*, not rotation. `DriveSubsystem` exploits this: it
tracks a running total, `m_virtualLeftDistanceMeters`, built each cycle from
the real right encoder's delta — added when the drivetrain is assumed to be
driving straight, subtracted when a command has flagged
`setTurningInPlace(true)`. That virtual value and the real right-side reading
are both passed to `m_odometry.update()`, which cannot tell the difference
between a real sensor and this reconstruction.

This two-branch version is only correct for exactly two motions (straight,
or a pure point turn) and is flagged as a to-do below to be replaced with a
general formula that works for any teleop motion, including the blended
forward-plus-turn arcade drive produces continuously.

## Simulation

Simulation is enabled. It uses a `DifferentialDrivetrainSim` physics model
driven by two `SparkMaxSim` instances (one per leader) and writes back through
the real CANcoder's and Pigeon 2's simulation state objects. The estimated
pose (from odometry) and the simulator's ground-truth pose are both plotted
on the same `Field2d` widget, as separate named objects, so the two can be
compared visually.

## Known limitations / To-do

- **[Priority] Odometry is only reliable for flagged motion, not general
  teleop driving.** `setTurningInPlace()` is never called anywhere in the
  current code, so the reconstruction always takes the "driving straight"
  branch. Any rotation introduced during ordinary arcade-drive teleop (the
  normal case) is not reflected in the virtual left distance, so the position
  estimate will drift whenever the driver turns.

  Fix: replace the flag entirely with the general reconstruction, which is
  exact for any combination of translation and rotation, not just the two
  flagged cases — `Δleft = Δright − trackWidth × Δheading` (sign flipped from
  the derivation in earlier discussion, since the real encoder here is on
  the right, not the left):

  ```java
  private Rotation2d m_lastHeading;  // must be set explicitly in the
                                      // constructor — Rotation2d has no
                                      // usable zero-value default the way a
                                      // double field does

  // in periodic(), replacing the current deltaRight/turning-flag block:
  final double deltaHeadingRadians = currentHeading.minus(m_lastHeading).getRadians();
  m_virtualLeftDistanceMeters += deltaRight - kDriveTrackWidthMeters * deltaHeadingRadians;
  m_lastHeading = currentHeading;
  ```

  Once this lands, `m_turningInPlace` and `setTurningInPlace()` are dead code
  and can be deleted. Note this still can't detect a fault specific to the
  unmeasured left side alone (a stalled motor, a slipping wheel) — it's
  derived from the right encoder and gyro, not independently sensed — and its
  accuracy during turns now depends on how well `kDriveTrackWidthMeters`
  matches the robot's real geometry, continuously rather than only during
  flagged autonomous turns.
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
  on heading error (remember continuous input across the ±180° wrap). These
  no longer need to call `setTurningInPlace()` once the general
  reconstruction above is in place — write them against the plain gyro
  reading and `drive.getPose()`, the same as any other autonomous command.
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
    the robot move plausibly on the field.
  - Run a "drive a square" or "drive out and back" routine and measure the
    closing error — how far the final estimated pose is from the expected
    end pose (ideally back at the start) — as a single summary number per
    run.
  - Deliberately mismatch `kTrackWidth` between the reconstruction and the
    simulator's construction to confirm turns (not straight segments) are
    where that error shows up, which is a good way to sanity-check the
    formula's sensitivity before trusting it on the real robot.
  - Once arcade-drive teleop is included in testing, compare estimated pose
    drift over a few minutes of mixed driving before and after switching to
    the general reconstruction formula, to quantify how much the current
    flag-only version actually costs in practice.
  - For a repeatable, dashboard-free check, consider a JUnit test that feeds
    a fixed, known sequence of encoder/gyro readings into `periodic()` and
    asserts the resulting pose against a hand-computed expected value.