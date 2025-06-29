package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

/**
 * The drivetrain subsystem. Manages the swerve drive including all modules, the gyro, kinematics, odometry, and system
 * identification.
 */
public class Drive extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
        AlertType.kError);

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    /**
     * If the trajectory following callback was run this tick. Reset at the end of each loop iteration so we know when
     * to continue pathing to the latest known pose.
     */
    private boolean trajectoryUpdatedThisTick = false;
    /** The latest trajectory target. See trajectoryUpdatedThisTick. If null, no trajectory has been followed yet. */
    private Pose2d latestTrajectoryTarget = null;

    /** If we're currently controlling the robot with velocity. */
    private boolean velocityControlMode = true;

    /** A debouncer that automatically unlocks the wheels after the robot has been disabled for a period of time. */
    private final Debouncer unlockWheelsDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kFalling);
    /** If the wheels are currently in brake mode. */
    private boolean wheelsLocked = true;

    private final PIDController xController = new PIDController(9.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(9.0, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(8.0, 1.5, 1.25);

    /**
     * Constructs a new Drive subsystem.
     * @param gyroIO
     * @param flModuleIO
     * @param frModuleIO
     * @param blModuleIO
     * @param brModuleIO
     * @param resetSimulationPoseCallBack A callback that will be called when the robot pose is updated, like at the
     *            start of autonomous. This is used to also reset the pose in simulation.
     */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, "FrontLeft");
        modules[1] = new Module(frModuleIO, "FrontRight");
        modules[2] = new Module(blModuleIO, "BackLeft");
        modules[3] = new Module(brModuleIO, "BackRight");

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            latestTrajectoryTarget = null;
        }));
    }

    /**
     * Creates a new auto factory for this drivetrain with the given trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        var robotState = RobotState.getInstance();
        return new AutoFactory(robotState::getPose, this::setPose, this::followPath, true, this, trajLogger);
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        trajectoryUpdatedThisTick = true;
        latestTrajectoryTarget = sample.getPose();

        var baseSpeeds = sample.getChassisSpeeds();

        double[] accelerations = new double[modules.length];
        for(int i = 0; i < modules.length; i++) {
            double accelerationX = sample.moduleForcesX()[i] / DriveConstants.robotMassKg;
            double accelerationY = sample.moduleForcesY()[i] / DriveConstants.robotMassKg;
            accelerations[i] = Math.sqrt(accelerationX * accelerationX + accelerationY * accelerationY);
        }

        followPathToTarget(latestTrajectoryTarget, accelerations, baseSpeeds);
    }

    public void followPathToTarget(Pose2d targetPose, double[] accelerations, ChassisSpeeds baseSpeeds) {
        var pose = RobotState.getInstance().getPose();

        Logger.recordOutput("Odometry/CurrentPose", pose);
        Logger.recordOutput("Odometry/TargetPose", targetPose);

        baseSpeeds.vxMetersPerSecond += xController.calculate(pose.getX(), targetPose.getX());
        baseSpeeds.vyMetersPerSecond += yController.calculate(pose.getY(), targetPose.getY());
        baseSpeeds.omegaRadiansPerSecond += thetaController.calculate(pose.getRotation().getRadians(),
            targetPose.getRotation().getRadians());

        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(baseSpeeds, RobotState.getInstance().getRotation()),
            accelerations);
    }

    /**
     * Runs the drive at the desired robot-relative velocity. Doesn't account for acceleration.
     * @param speeds
     * @param accelerations
     */
    public void runVelocity(ChassisSpeeds speeds) {
        runVelocity(speeds, new double[4]);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds, double[] accelerationsMps2) {
        Logger.recordOutput("SwerveChassisSpeeds/TargetSpeeds", speeds);

        velocityControlMode = true;

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

        // Send setpoints to modules
        for(int i = 0; i < 4; i++) modules[i].runSetpoint(setpointStates[i], accelerationsMps2[i]);

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        velocityControlMode = false;
        for(int i = 0; i < 4; i++) modules[i].runCharacterization(output);
    }

    /** Runs the drive to rotate with the specified drive output for angular system identification. */
    public void runAngularCharacterization(double output) {
        velocityControlMode = false;
        for(int i = 0; i < 4; i++) modules[i].runAngularCharacterization(output);
    }

    /** Runs a particular module in a straight line with the specified drive output. */
    public void runCharacterization(int module, double output) {
        velocityControlMode = false;
        modules[module].runCharacterization(output);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Resets the modules to absolute. */
    public void resetToAbsolute() {
        for(int i = 0; i < 4; i++) modules[i].resetToAbsolute();
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for(int i = 0; i < 4; i++) headings[i] = moduleTranslations[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) states[i] = modules[i].getState();
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) states[i] = modules[i].getPosition();
        return states;
    }

    /**
     * Resets the odometry to the specified pose. Used at the start of autonomous to tell the robot where it is.
     * @return
     */
    public void setPose(Pose2d pose) {
        RobotState.getInstance().setPose(pose, getModulePositions());
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the measured linear speed of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredLinear")
    public double getLinearSpeedMetersPerSec() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.sqrt(
            speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for(int i = 0; i < 4; i++) values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for(int i = 0; i < 4; i++) output += modules[i].getFFCharacterizationVelocity() / 4.0;
        return output;
    }

    /** Returns the drive motor current draw of a particular module in amps. */
    public double getSlipMeasurementCurrent(int module) {
        return modules[module].getSlipMeasurementCurrent();
    }

    /** Returns the drive motor position of a particular module in radians. */
    public double getSlipMeasurementPosition(int module) {
        return modules[module].getWheelRadiusCharacterizationPosition();
    }

    /** Temporarily changes the drive motor current limit for slip current measurement. */
    public void setSlipMeasurementCurrentLimit(int amps) {
        for(int i = 0; i < 4; i++) modules[i].setSlipMeasurementCurrentLimit(amps);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for(var module : modules) module.periodic();
        odometryLock.unlock();

        // Stop moving when disabled
        if(DriverStation.isDisabled()) {
            for(var module : modules) module.stop();
        }

        // Unlock wheels if we've been disabled for a while
        boolean shouldLock = unlockWheelsDebouncer.calculate(DriverStation.isEnabled());
        if(shouldLock != wheelsLocked) {
            wheelsLocked = shouldLock;
            for(var module : modules) module.setBrakeMode(wheelsLocked);
        }

        // Log empty setpoint states when disabled
        if(DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        var robotState = RobotState.getInstance();
        for(int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            robotState.applyOdometryUpdate(sampleTimestamps[i], modulePositions,
                gyroInputs.connected ? Optional.of(gyroInputs.odometryYawPositions[i]) : Optional.empty());
        }
        robotState.addDriveSpeeds(getChassisSpeeds());

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        if(!trajectoryUpdatedThisTick && velocityControlMode && latestTrajectoryTarget != null
            && DriverStation.isAutonomousEnabled()) {
            followPathToTarget( //
                latestTrajectoryTarget, //
                new double[modules.length], //
                new ChassisSpeeds(0., 0., 0.) //
            );
        }
        trajectoryUpdatedThisTick = false;
    }
}
