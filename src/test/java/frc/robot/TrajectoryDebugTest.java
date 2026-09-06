// src/test/java/frc/robot/TrajectoryDebugTest.java
package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import org.junit.jupiter.api.Test;

class TrajectoryDebugTest {
    @Test
    void printTrajectory1States() {
        Trajectory trajectory = Trajectories.generateTrajectory1();
        for (Trajectory.State state : trajectory.getStates()) {
            System.out.printf(
                "t=%5.2fs  x=%6.3f  y=%6.3f  heading=%7.2f°  v=%5.2f m/s  curvature=%6.3f rad/m%n",
                state.timeSeconds,
                state.poseMeters.getX(),
                state.poseMeters.getY(),
                state.poseMeters.getRotation().getDegrees(),
                state.velocityMetersPerSecond,
                state.curvatureRadPerMeter);
        }
    }
}