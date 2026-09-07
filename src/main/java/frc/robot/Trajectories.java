package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.kDriveTrackWidthMeters;
import static frc.robot.Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
    
    public static Trajectory generateTrajectory1() {

        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kDriveTrackWidthMeters);

        var startPose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(0.0));

        var endPose  =new Pose2d(Units.feetToMeters(20.0), Units.feetToMeters(10), Rotation2d.fromDegrees(90.0));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(9.0), Units.feetToMeters(-8.0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(13.0), Units.feetToMeters(-10.0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(16.0), Units.feetToMeters(-8.0)));


        TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kinematics)
            .addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, kMaxSpeedMetersPerSecond));
        
        

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            interiorWaypoints,
            endPose,
            config);

        return trajectory;
    }


    public static Trajectory generateTrajectory2() {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kDriveTrackWidthMeters);

        TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kinematics)
            .addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, kMaxSpeedMetersPerSecond));

        var startPose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), Rotation2d.fromDegrees(-90.0));

        var endPose  =new Pose2d(Units.feetToMeters(20.0), Units.feetToMeters(0), Rotation2d.fromDegrees(-90));

        return TrajectoryGenerator.generateTrajectory(
            List.of(
                startPose,
                new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(-5.0), Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(0.0), Rotation2d.fromDegrees(90.0)),
                new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(0.0)),
                //new Pose2d(Units.feetToMeters(.0), Units.feetToMeters(0.0), Rotation2d.fromDegrees(90.0)),
                endPose),
            config);
        
    }
}
