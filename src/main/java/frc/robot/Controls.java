package frc.robot;

import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;

public class Controls {
    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)",
        AlertType.kWarning);
    private final Alert operatorDisconnectedAlert = new Alert("Operator controller disconnected (port 1)",
        AlertType.kWarning);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(Constants.twoDriverMode ? 1 : 0);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 20.0);

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
    }

    /** Configures the controls. */
    public void configureControls(Drive drive, SwerveDriveSimulation driveSimulation, Roller roller, Vision vision) {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.isSim ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(RobotState.getInstance().getPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero
                    : Rotation2d.k180deg)); // Zero gyro
        final Runnable resetOdometry = Constants.isSim
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(
                new Pose2d(0, 0, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero
                    : Rotation2d.k180deg)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        driver.start().and(driver.leftStick()).debounce(0.5)
            .onTrue(Commands.runOnce(resetOdometry, drive).ignoringDisable(true));
        // Used to account for swerve belts slipping.
        driver.back().whileTrue(Commands.run(drive::resetToAbsolute));

        // Default roller command, control with triggers
        roller.setDefaultCommand(
            roller.runTeleop(() -> operator.getRightTriggerAxis() * 0.75, () -> operator.getLeftTriggerAxis() * 0.75));

        operator.a().whileTrue(roller.runPercent(0.5));

        driver.b()
            .whileTrue(Commands.defer(() -> DriveCommands.scoreAtActiveBranch(drive, roller), Set.of(drive, roller)));

        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(true, true, RumbleType.kBothRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(true, true, RumbleType.kBothRumble).withTimeout(0.4)
            .andThen(Commands.waitSeconds(0.3)).repeatedly().withTimeout(2.0));
    }

    private HashMap<Integer, Double> driverRumbleCommands = new HashMap<>();
    private HashMap<Integer, Double> operatorRumbleCommands = new HashMap<>();

    public void setDriverRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            driverRumbleCommands.remove(hash);
        } else {
            driverRumbleCommands.put(hash, value);
        }
        driver.setRumble(type, driverRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public void setOperatorRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            operatorRumbleCommands.remove(hash);
        } else {
            operatorRumbleCommands.put(hash, value);
        }
        operator.setRumble(type, operatorRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public Command controllerRumbleWhileRunning(boolean forDriver, boolean forOperator, RumbleType type) {
        return Commands.startEnd(() -> {
            if(forDriver) setDriverRumble(type, 1.0, hashCode());
            if(forOperator) setOperatorRumble(type, 1.0, hashCode());
        }, () -> {
            if(forDriver) setDriverRumble(type, 0.0, hashCode());
            if(forOperator) setOperatorRumble(type, 0.0, hashCode());
        }).withName("ControllerRumbleWhileRunning");
    }

    /** Updates the controls, including shown alerts. */
    public void update() {
        // Controller disconnected alerts
        int driverPort = driver.getHID().getPort();
        int operatorPort = operator.getHID().getPort();
        driverDisconnectedAlert
            .set(!DriverStation.isJoystickConnected(driverPort) || !DriverStation.getJoystickIsXbox(driverPort));
        operatorDisconnectedAlert
            .set(!DriverStation.isJoystickConnected(operatorPort) || !DriverStation.getJoystickIsXbox(operatorPort));
    }
}
