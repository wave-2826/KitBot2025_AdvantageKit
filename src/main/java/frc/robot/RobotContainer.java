package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.drive.DriveTuningCommands;
import frc.robot.commands.vision.VisionTuningCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonSRX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private final Roller roller;

    // Only used in simulation
    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final Alert logReceiverQueueAlert = new Alert(
        "Logging queue exceeded capacity; data isn't being logged! This may fix itself.", AlertType.kError);
    private final Alert noAutoSelectedAlert = new Alert("No auto selected!", AlertType.kWarning);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch(Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(new GyroIONavX(), new ModuleIOSpark(DriveConstants.frontLeftModule),
                    new ModuleIOSpark(DriveConstants.frontRightModule),
                    new ModuleIOSpark(DriveConstants.backLeftModule),
                    new ModuleIOSpark(DriveConstants.backRightModule));
                roller = new Roller(new RollerIOTalonSRX());
                var robotState = RobotState.getInstance();
                vision = new Vision(new VisionIOLimelight(VisionConstants.leftCameraHostname, robotState::getRotation),
                    new VisionIOLimelight(VisionConstants.rightCameraHostname, robotState::getRotation));
                break;

            case SIM:
                // Create a maple-sim swerve drive simulation instance
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                    new Pose2d(2, 2, Rotation2d.kZero));
                // Add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]), new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]), new ModuleIOSim(driveSimulation.getModules()[3]));

                roller = new Roller(new RollerIOSim());
                vision = new Vision(
                    new VisionIOPhotonVisionSim(VisionConstants.leftCameraHostname, VisionConstants.leftCameraPos,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.rightCameraHostname, VisionConstants.rightCameraPos,
                        driveSimulation::getSimulatedDriveTrainPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(new GyroIO() {
                }, new ModuleIO() {
                }, new ModuleIO() {
                }, new ModuleIO() {
                }, new ModuleIO() {
                });
                roller = new Roller(new RollerIO() {
                });
                vision = new Vision(new VisionIO() {
                }, new VisionIO() {
                });
                break;
        }

        // Set up auto routines
        NamedCommands.registerCommand("Score", roller.runPercent(0.5).withTimeout(1.5));

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("Center 1 Coral"));

        double distanceInches = 90 - 3 - 5;
        double timeSeconds = 2.5;
        autoChooser.addOption("wtf is pathplanner doing",
            Commands.sequence(
                DriveCommands.driveStraightCommand(drive, Units.inchesToMeters(distanceInches / timeSeconds),
                    RobotState.getInstance()::getRotation).withTimeout(timeSeconds),
                roller.runPercent(0.5).withTimeout(1.5)));

        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, autoChooser);
        VisionTuningCommands.addTuningCommandsToAutoChooser(vision, autoChooser);

        // Configure the button bindings
        Controls.getInstance().configureControls(drive, driveSimulation, roller, vision);
    }

    public void updateAlerts() {
        String selected = autoChooser.getSendableChooser().getSelected();
        noAutoSelectedAlert.set(DriverStation.isDisabled() && (selected == null || selected == "None"));

        logReceiverQueueAlert.set(Logger.getReceiverQueueFault());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void resetSimulatedRobot() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, Rotation2d.kZero));
    }

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

        // if(!VisionConstants.enableVisionSimulation) {
        //     RobotState.getInstance().addVisionMeasurement(driveSimulation.getSimulatedDriveTrainPose(),
        //         Timer.getTimestamp(), VecBuilder.fill(0, 0, 0));
        // }
    }
}
