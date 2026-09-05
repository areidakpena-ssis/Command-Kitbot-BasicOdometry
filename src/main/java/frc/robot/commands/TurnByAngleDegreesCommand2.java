// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnByAngleDegreesCommand2 extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final DriveSubsystem m_driveSubsystem;
  private final double MAX_TURN_SPEED = 0.8;
  private final Rotation2d m_turnTargetHeading;
  private double m_deltaDegrees;
  private double m_deltaDegreesInitial;
  private double m_turnSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnByAngleDegreesCommand2(DriveSubsystem driveSubsystem, double deltaDegrees) {
    m_driveSubsystem = driveSubsystem;
    m_turnTargetHeading = m_driveSubsystem.getHeadingRotation2d().plus(Rotation2d.fromDegrees(deltaDegrees));
    m_deltaDegrees = deltaDegrees;
    m_deltaDegreesInitial = deltaDegrees;
    m_turnSpeed = MAX_TURN_SPEED;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.setTurningInPlace(true);
    m_turnSpeed = (m_deltaDegrees >= 0)?  m_turnSpeed : -m_turnSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_deltaDegrees = m_driveSubsystem.getHeadingRotation2d().minus(m_turnTargetHeading).getDegrees(); 
    // update turn speed proportionally
    m_turnSpeed = Math.abs(m_deltaDegrees / m_deltaDegreesInitial) * MAX_TURN_SPEED;
    m_driveSubsystem.tankDriveNoSquare(-m_turnSpeed, m_turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setTurningInPlace(false);
    m_driveSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(
      m_driveSubsystem.getHeadingRotation2d().minus(m_turnTargetHeading).getDegrees()) 
        < 2.0;
  }
}
