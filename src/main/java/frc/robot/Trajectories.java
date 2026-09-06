package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

import java.util.ArrayList;

public class Trajectories {
    
    public static Trajectory generateTrajectory1() {
        var startPose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), Rotation2d.fromDegrees(0.0));

        var endPose  =new Pose2d(Units.feetToMeters(20.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(90.0));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.0), Units.feetToMeters(-5.0)));

        TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            interiorWaypoints,
            endPose,
            config);

        return trajectory;
    }
}
