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

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathFindConstants;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.ReefTarget;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {}

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runOpenLoop(voltage, voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  public static Command PathfindtoBranch(Drive drive) {
    return Commands.run(
        () -> {
          Pose2d targetPose = getBranchPose(DriverStationInterface.getInstance().getReefTarget());

          // Create the constraints to use while pathfinding
          PathConstraints constraints =
              new PathConstraints(
                  3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

          // Since AutoBuilder is configured, we can use it to build pathfinding commands
          Command pathfindingCommand =
              AutoBuilder.pathfindToPose(
                  targetPose, constraints, 0.0 // Goal end velocity in meters/sec
                  );
          if (targetPose == new Pose2d(0, 0, Rotation2d.fromDegrees(0))) {
            System.out.println("Invalid target pose for pathfinding to branch.");
          }

          Command RotationCommand =
              Commands.run(
                      () -> {
                        PIDController rotationController =
                            new PIDController(
                                PathFindConstants.kP, PathFindConstants.kI, PathFindConstants.kD);
                        rotationController.enableContinuousInput(-180.0, 180.0);
                        double targetAngle = targetPose.getRotation().getDegrees();
                        double currentAngle = drive.getRotation().getDegrees();
                        double rotationOutput =
                            rotationController.calculate(currentAngle, targetAngle);
                        rotationOutput = MathUtil.clamp(rotationOutput, -1.0, 1.0); // Clamp output

                        // turn the robot to face the target pose in the nearest direction
                        rotationOutput = MathUtil.applyDeadband(rotationOutput, DEADBAND);
                        rotationOutput = MathUtil.clamp(rotationOutput, -1.0, 1.0);
                        // Run the drive with the calculated rotation output

                        double leftOutput = -rotationOutput * PathFindConstants.maxRotationSpeed;
                        double rightOutput = rotationOutput * PathFindConstants.maxRotationSpeed;

                        drive.runOpenLoop(leftOutput, rightOutput);
                        rotationController.close();
                      })
                  .until(
                      () ->
                          Math.abs(
                                  drive.getRotation().getDegrees()
                                      - targetPose.getRotation().getDegrees())
                              < 5.0);

          Commands.sequence(pathfindingCommand, RotationCommand)
              .schedule(); // Schedule the pathfinding command followed by the rotation command
        },
        drive);
  }

  private static Pose2d getBranchPose(ReefTarget reefTarget) {
    switch (reefTarget.branch()) {
      case A:
        return PathFindConstants.A;
      case B:
        return PathFindConstants.B;
      case C:
        return PathFindConstants.C;
      case D:
        return PathFindConstants.D;
      case E:
        return PathFindConstants.E;
      case F:
        return PathFindConstants.F;
      case G:
        return PathFindConstants.G;
      case H:
        return PathFindConstants.H;
      case I:
        return PathFindConstants.I;
      case J:
        return PathFindConstants.J;
      case K:
        return PathFindConstants.K;
      case L:
        return PathFindConstants.L;
      default:
        return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }
  }
}
