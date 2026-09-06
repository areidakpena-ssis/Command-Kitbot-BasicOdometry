// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import frc.robot.commands.TurnByAngleDegreesCommand2;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    /** Drive a fixed distance
     * 
     */
    public static Command driveDistanceAuto(DriveSubsystem driveSubsystem) {

        final double[] startDistance = new double[1];  // mutable box — a plain local can't be reassigned inside a lambda

        return Commands.sequence(
            Commands.runOnce(() -> startDistance[0] = driveSubsystem.getAverageDistanceMeters()),

            driveSubsystem.arcadeDriveCommand(()->0.4, ()->0.0)
                .until(() -> driveSubsystem.getAverageDistanceMeters() - startDistance[0] 
                        > AutoConstants.kDistanceTargetMeters)
                .finallyDo(() -> driveSubsystem.stopMotors())
        );
    }

    public static Command driveTurnDriveAuto(DriveSubsystem driveSubsystem) {
        return Commands.sequence(
            driveDistanceAuto(driveSubsystem),
            Commands.waitSeconds(1.0),
            //driveSubsystem.turnByAngleDegreesCommand(90),
            new TurnByAngleDegreesCommand2(driveSubsystem, 90),
            Commands.waitSeconds(1.0),
            driveDistanceAuto(driveSubsystem)
        );
    }

    public static Command Trajectory1Auto(DriveSubsystem driveSubsystem) {
        Trajectory trajectory1 = Trajectories.generateTrajectory1();

        return Commands.runOnce(() -> driveSubsystem.resetOdometry(trajectory1.getInitialPose()))
            .andThen(new FollowTrajectory(driveSubsystem, trajectory1));

    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
