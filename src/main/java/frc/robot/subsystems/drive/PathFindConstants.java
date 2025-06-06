package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathFindConstants {

  // Reef Positions
  public static final Pose2d A = new Pose2d(3.19, 4.28, Rotation2d.fromDegrees(0));
  public static final Pose2d B = new Pose2d(3.19, 3.77, Rotation2d.fromDegrees(0));
  public static final Pose2d C = new Pose2d(3.66, 3.01, Rotation2d.fromDegrees(60));
  public static final Pose2d D = new Pose2d(4.04, 2.82, Rotation2d.fromDegrees(60));
  public static final Pose2d E = new Pose2d(4.92, 2.75, Rotation2d.fromDegrees(120));
  public static final Pose2d F = new Pose2d(5.32, 3.03, Rotation2d.fromDegrees(120));
  public static final Pose2d G = new Pose2d(5.79, 3.78, Rotation2d.fromDegrees(180));
  public static final Pose2d H = new Pose2d(5.79, 4.23, Rotation2d.fromDegrees(180));
  public static final Pose2d I = new Pose2d(5.33, 5.05, Rotation2d.fromDegrees(-120));
  public static final Pose2d J = new Pose2d(4.93, 5.30, Rotation2d.fromDegrees(-120));
  public static final Pose2d K = new Pose2d(3.93, 5.28, Rotation2d.fromDegrees(-60));
  public static final Pose2d L = new Pose2d(3.61, 5.02, Rotation2d.fromDegrees(-60));

  // PID Constants
  public static final double kP = 0.1; // Proportional gain
  public static final double kI = 0.0; // Integral gain
  public static final double kD = 0.0; // Derivative gain
  public static final double maxRotationSpeed = 3.0; // Maximum rotation speed in m/s
  }
}
