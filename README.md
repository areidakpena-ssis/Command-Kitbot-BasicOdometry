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

## Simulation

Simulation is enabled. It uses a `DifferentialDrivetrainSim` physics model
driven by two `SparkMaxSim` instances (one per leader) and writes back through
the real CANcoder's and Pigeon 2's simulation state objects. The estimated
pose (from odometry) and the simulator's ground-truth pose are both plotted
on the same `Field2d` widget, as separate named objects, so the two can be
compared visually.

## Known limitations / To-do

- **Odometry is only reliable for flagged motion, not general teleop
  driving.** `setTurningInPlace()` is never called anywhere in the current
  code, so the reconstruction always takes the "driving straight" branch. Any
  rotation introduced during ordinary arcade-drive teleop (the normal case)
  is not reflected in the virtual left distance, so the position estimate
  will drift whenever the driver turns. Priority fix: replace the two-branch
  flag with the general reconstruction, which needs no flag at all —
  `Δvirtual = Δreal + trackWidth × Δheading` (or the opposite sign, since the
  real encoder is on the right here) — since it's exact for any combination
  of translation and rotation, not just the two flagged cases.
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
  are also the first commands that would actually call
  `setTurningInPlace(true)`/`(false)`, which nothing in the current code does
  — wiring one up is a prerequisite for the odometry reconstruction fix above
  to matter in practice.
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