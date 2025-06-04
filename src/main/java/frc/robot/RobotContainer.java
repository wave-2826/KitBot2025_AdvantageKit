// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonSRX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Roller roller;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonSRX(), new GyroIONavX());
        roller = new Roller(new RollerIOTalonSRX());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        // vision =
        // new Vision(
        // Drive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0));
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        roller = new Roller(new RollerIOSim());
        vision = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        roller = new Roller(new RollerIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("Score", roller.runPercent(1.0).withTimeout(3.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default drive command, normal arcade drive
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> controller.getRightX(), () -> controller.getLeftY()));

    // Default roller command, control with triggers
    roller.setDefaultCommand(
        roller.runTeleop(
            () -> controller.getRightTriggerAxis(), () -> controller.getLeftTriggerAxis()));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                  } else {
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg));
                  }
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
