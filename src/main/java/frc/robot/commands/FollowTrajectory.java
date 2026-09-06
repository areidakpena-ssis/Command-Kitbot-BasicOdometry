// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FollowTrajectory extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")

    private static final double kPoseToleranceMeters = 0.05;
    private static final double kHeadingToleranceRadians = Math.toRadians(2.0);
    private static final double kMaxSettleSeconds = 2.0; // max time beyond end of trajectory timer

    private final DriveSubsystem m_driveSubsystem;
    private final Trajectory m_trajectory;
    private final Timer m_timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTrajectory(DriveSubsystem drive, Trajectory trajectory) {
    m_driveSubsystem = drive;
    m_trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.driveToTrajectoryState(m_trajectory.sample(m_timer.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds())) {
      return false;
    }

    Pose2d finalPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
    Pose2d currentPose = m_driveSubsystem.getPose();

    boolean settled = 
      currentPose.getTranslation().getDistance(finalPose.getTranslation()) < kPoseToleranceMeters
      && Math.abs(currentPose.getRotation().minus(finalPose.getRotation()).getRadians()) < kHeadingToleranceRadians;

    return settled 
            || m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds() + kMaxSettleSeconds);
  }
}
