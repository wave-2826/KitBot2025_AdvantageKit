package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.auto.AutoRoutines;
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
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIORio;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonSRX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedAutoChooser;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

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
    private final LEDs leds;

    // Only used in simulation
    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    @SuppressWarnings("unused")
    private final AutoRoutines routines;
    private final LoggedAutoChooser autoChooser;

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
                leds = new LEDs(new LEDIORio());
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
                RobotState.getInstance().resetSimulationPoseCallback = driveSimulation::setSimulationWorldPose;

                roller = new Roller(new RollerIOSim());
                vision = new Vision(
                    new VisionIOPhotonVisionSim(VisionConstants.leftCameraHostname, VisionConstants.leftCameraPos,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.rightCameraHostname, VisionConstants.rightCameraPos,
                        driveSimulation::getSimulatedDriveTrainPose));

                leds = new LEDs(new LEDIOSim());
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

                leds = new LEDs(new LEDIO() {
                });
                break;
        }

        autoChooser = new LoggedAutoChooser("Auto Choices");
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        routines = new AutoRoutines(drive, roller, autoChooser);

        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, autoChooser);
        VisionTuningCommands.addTuningCommandsToAutoChooser(vision, autoChooser);

        // Configure the button bindings
        Controls.getInstance().configureControls(drive, driveSimulation, roller, vision);

        leds.configureControls();
    }

    public void updateAlerts() {
        String selected = autoChooser.getSelectedName();
        noAutoSelectedAlert.set(DriverStation.isDisabled() && (selected == null || selected == "None"));

        logReceiverQueueAlert.set(Logger.getReceiverQueueFault());
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
