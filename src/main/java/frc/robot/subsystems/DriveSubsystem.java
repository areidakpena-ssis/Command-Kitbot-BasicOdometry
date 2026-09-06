// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class DriveSubsystem extends SubsystemBase {
    // --- motors ---
    private final SparkMax m_leftLeader = new SparkMax(kLeftLeaderId, MotorType.kBrushed);
    private final SparkMax m_leftFollower = new SparkMax(kLeftFollowerId, MotorType.kBrushed);
    private final SparkMax m_rightLeader = new SparkMax(kRightLeaderId, MotorType.kBrushed);
    private final SparkMax m_rightFollower = new SparkMax(kRightFollowerId, MotorType.kBrushed);

    // --- Differential Drive subsystem 
    private final DifferentialDrive m_differentialDrive;

    // --- Encoders ---
    private final CANcoder m_rightEncoder = new CANcoder(kRightEncoderID);

    // --- Gryo ----
    private final Pigeon2 m_pigeon2 = new Pigeon2(0);  // need to install on CAN bus

    // --- Odometry ---
    private final DifferentialDriveOdometry m_odometry;
     // No physical left encoder exists. This is a running total built one
    // signed delta at a time, so it stays continuous across mode switches.
    private double m_virtualLeftDistanceMeters = 0.0;
    private double m_lastRightDistanceMeters;
    // Set by whichever command currently owns the drivetrain.
    private boolean m_turningInPlace = false;

    // --- kinematics and trajectory following
    private final LTVUnicycleController m_controller = new LTVUnicycleController(0.020);
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kDriveTrackWidthMeters);


    private final Field2d m_field = new Field2d();
    // simulation field for diagnostic
    private final Field2d m_SimModelField = new Field2d();

    // helpers for turning and keeping track of state
    private Rotation2d m_turnTargetHeading;

    // --- Simulation setup --- 
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(
        kDriveBase_kV, kDriveBase_kA, 
        kDriveBase_kVAngular, kDriveBase_kAAngular, 
        kDriveTrackWidthMeters);

    private final DifferentialDrivetrainSim m_drivetrainSimulator = 
        new DifferentialDrivetrainSim(
            m_drivetrainSystem,
            DCMotor.getCIM(2),
            kDriveGearRatio,
            kDriveTrackWidthMeters,
            kWheelDiameterMeters/2.0,
            null);

    // handles:  both leaders needs a SparkMaxSim, even though nothing reads
    // their positions, because iterate is what refreshed getAppliedOutput()
    private final SparkMaxSim m_leftLeaderSim = new SparkMaxSim(m_leftLeader, DCMotor.getCIM(2));
    private final SparkMaxSim m_rightLeaderSim = new SparkMaxSim(m_rightLeader, DCMotor.getCIM(2));
    private CANcoderSimState m_rightEncoderSim;
    private Pigeon2SimState m_gyroSim;


    /** Creates a new DriveSubsystem, configuring the KitBot's differential drivetrain motors and right-side CANcoder. */
    public DriveSubsystem() {
        
        // Shared motor configs
        SparkMaxConfig sharedMotorConfig = new SparkMaxConfig();
        sharedMotorConfig.voltageCompensation(12);
        sharedMotorConfig.smartCurrentLimit(kDriveMotorCurrentLimit);

        // Left leader: invert so that positive values drive both sides forward
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.apply(sharedMotorConfig);
        leftConfig.inverted(kLeftLeaderReversed);
        m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right leader: not inverted
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.apply(sharedMotorConfig);
        rightConfig.inverted(kRightLeaderReversed); // should be False
        m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Followers mirror their respective leaders
        leftConfig.follow(m_leftLeader);
        m_leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        rightConfig.follow(m_rightLeader);
        m_rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // rest encoders and gyro and initilaize odometry object.
        resetEncoders();
        m_pigeon2.reset();
        m_odometry = new DifferentialDriveOdometry(
            m_pigeon2.getRotation2d(), 
            m_virtualLeftDistanceMeters, getRightDistanceMeters()); // should be 0.0, 0,0

        m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);


        if (RobotBase.isSimulation()) {
            m_rightEncoderSim = m_rightEncoder.getSimState();
            m_gyroSim = m_pigeon2.getSimState();
        }

        SmartDashboard.putData("Field", m_field);
    }

    
    /**
     * A split-stick arcade command, with forward/backward controlled by the left hand, 
     * and turning controlled by right; values should be negated when sent from controller
     * due to differences in coordinate orientation. 
     * @param fwd 
     * @param rot
     * @return
     */
    public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
        return run(() -> m_differentialDrive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
            .withName("arcadeDrive");
    }

    public void tankDriveNoSquare(double leftSpeed, double rightSpeed) {
        m_differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
    }

    public Command stopDriveCommand() {
        return runOnce( () -> this.stopMotors() );
    }

    public Command resetEncoderCommand() {
        return runOnce( () -> this.resetEncoders() );
    }

    /**
     * Returns the distance traveled by the left side in meters.
     * Negated so that forward motion (left motor inverted) gives positive distance.
    public double getLeftDistanceMeters() {
        return -m_leftEncoder.getPosition() * kDistancePerRotationMeters;
    }
     */

    /**
     * Returns the distance traveled by the right side in meters.
     */
    @Logged
    public double getRightDistanceMeters() {
        return m_rightEncoder.getPosition().getValueAsDouble() * kDistancePerRotationMeters;
    }


    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    public Rotation2d getHeadingRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    @Logged
    public double getHeadingDegrees() {
        return m_pigeon2.getRotation2d().getDegrees();
    }

    


    /**
     * Returns the average distance traveled by both sides in meters.
     * Convenient for straight-line distance calculations in auto.
     *
     * <p>Only one CANcoder is physically installed (right side) as of this build, so this
     * currently just returns the right-side distance rather than a true left/right average.
     * This is a temporary placeholder — Tier 3 Task 3 has students install a second encoder
     * and wire it in here for real two-sided tracking.
     */
    @Logged
    public double getAverageDistanceMeters() {
        //return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
        return getRightDistanceMeters();
    }

    /** Returns current velocity estimate in m/s based on the CANcoder 
     * that is mounted directly to the output axel. 
     */
    @Logged()
    public double getVelocityMetersPerSecond() {
        return m_rightEncoder.getVelocity().getValueAsDouble() * kDistancePerRotationMeters; 
    }

    /** Resets both drive encoders to zero. */
    public void resetEncoders() {
        //m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        m_virtualLeftDistanceMeters = 0.0;
    }

    /**
     * The one place this subsystem must accept an absolute pose from outside.
     * Add a guarded reset of the physics model here once simulation is added,
     * or the estimate and the simulated robot will disagree permanently.
     */

    public void resetOdometry(Pose2d pose) {
        // check for simulation mode
        if (RobotBase.isSimulation()) {
            m_drivetrainSimulator.setPose(pose);
        }

        m_odometry.resetPosition(m_pigeon2.getRotation2d(), 
            m_rightEncoder.getPosition().getValueAsDouble(),
            m_virtualLeftDistanceMeters,
            pose);
    }

    /** 
     * Stops drivetrain motors immediately
     */
    public void stopMotors() {
        m_differentialDrive.stopMotor();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        // update odometry

        // get existing encoder readings: 
        final Rotation2d currentHeading = m_pigeon2.getRotation2d();
        final double currentRight = getRightDistanceMeters();

        // compute deltaRight (for purpose of faking deltaLeft)
        final double deltaRight = currentRight - m_lastRightDistanceMeters;
        m_lastRightDistanceMeters = currentRight;
        // update virtual left distance; note the +=
        m_virtualLeftDistanceMeters += m_turningInPlace ? -deltaRight : deltaRight;

        m_odometry.update(currentHeading, m_virtualLeftDistanceMeters, currentRight);

        SmartDashboard.putNumber("Odometry/X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/HeadingDeg", m_odometry.getPoseMeters().getRotation().getDegrees());

        m_field.setRobotPose(m_odometry.getPoseMeters()); // estimate
        //m_field.getObject("Odometry").setPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
        double batteryVoltage = RoboRioSim.getVInVoltage();
        m_rightEncoderSim.setSupplyVoltage(batteryVoltage);
        m_gyroSim.setSupplyVoltage(batteryVoltage);

        // Step 1: determine voltages from applied duty cycle:
        // duty cycle is fraction of battery voltage
        double leftVolts = m_leftLeader.getAppliedOutput() * batteryVoltage;
        double rightVolts = m_rightLeader.getAppliedOutput() * batteryVoltage;

        // update state space model
        m_drivetrainSimulator.setInputs(leftVolts, rightVolts);
        m_drivetrainSimulator.update(0.02); // 50 Hz

         // sparkmax internal encoder is rotor side
        m_leftLeaderSim.iterate(mpsToMotorRpm(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()),
                          batteryVoltage, 0.020);
        m_rightLeaderSim.iterate(mpsToMotorRpm(m_drivetrainSimulator.getRightVelocityMetersPerSecond()),
                           batteryVoltage, 0.020);

        // cancoders on axel
        m_rightEncoderSim.setRawPosition(
            m_drivetrainSimulator.getRightPositionMeters() / kDistancePerRotationMeters);

        m_rightEncoderSim.setVelocity(
            m_drivetrainSimulator.getRightVelocityMetersPerSecond() / kDistancePerRotationMeters);

        m_gyroSim.setRawYaw(m_drivetrainSimulator.getHeading().getDegrees());

        // get the simulated robot pose from the simulation
        // Use to verify the odometry estimates via simulation
        m_field.getObject("Simulation model").setPose(m_drivetrainSimulator.getPose());
    }



    // helper function: convert speed in meters per second to RPM
    private static double mpsToMotorRpm(double mps) {
        return mps / kDistancePerRotationMeters * kDriveGearRatio * 60.0;
    }


    /* Helper methods for setting drive commands in meters per second */
    private final SimpleMotorFeedforward m_feedforward = 
        new SimpleMotorFeedforward(kDriveBase_kS, kDriveBase_kV, kDriveBase_kA); // Replace with your actual constants


    /**
     * Commands the robot to drive at a specific target velocity using only feedforward.
     * 
     * @param targetVelocityMetersPerSecond The desired speed in m/s
     */
    public Command testFeedforwardCommand(double targetVelocityMetersPerSecond) {
        return run(() -> {
            // Calculate the required voltage to achieve the target velocity
            double appliedVoltage = m_feedforward.calculate(targetVelocityMetersPerSecond);

            // Bypass DifferentialDrive and apply the voltage directly to the motor controllers
            m_leftLeader.setVoltage(appliedVoltage);
            m_rightLeader.setVoltage(appliedVoltage);

            // Feed the Motor Safety watchdog so it doesn't disable our motors
            m_differentialDrive.feed();
        })
        .finallyDo(() -> this.stopMotors())
        .withName("testFeedforward");
    }

    /** Turning mechanisms for auto routines and odometry*/
    public void setTurningInPlace(boolean turning) {
        m_turningInPlace = turning;
    }


    public void setSpeeds(double leftVelocityMetersPerSecond, double rightVelocityMetersPerSecond) {
        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond);

        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);
    }


    public void setSpeeds(DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds) {

        double leftVelocityMetersPerSecond = differentialDriveWheelSpeeds.leftMetersPerSecond;
        double rightVelocityMetersPerSecond = differentialDriveWheelSpeeds.rightMetersPerSecond;

        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond);

        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);
    }



    /**
     * Turn by fixed angle in degrees from current heading, CCW positive; 
     */
    public Command turnByAngleDegreesCommand(double deltaDegrees) {
        return runOnce( () -> {
            m_turnTargetHeading = getHeadingRotation2d().plus(Rotation2d.fromDegrees(deltaDegrees));
        }).andThen(
            run( () -> {
                double speed = (deltaDegrees >= 0)?  0.3 : -0.3;
                m_turningInPlace = true;
                m_differentialDrive.tankDrive(-speed, speed, false);
            }).until(
                () -> Math.abs(getHeadingRotation2d().minus(m_turnTargetHeading).getDegrees()) < 1.0)
        ).finallyDo(interrupted -> {
            stopMotors();
            m_turningInPlace = false;
            }
        );
    }


    /** Drives one sampled trajectory state. Called once per scheduler pass by a following command. */
    public void driveToTrajectoryState(Trajectory.State desiredState) {
        ChassisSpeeds targetSpeeds = m_controller.calculate(m_odometry.getPoseMeters(), desiredState);
        setSpeeds(m_kinematics.toWheelSpeeds(targetSpeeds));
    }

}
