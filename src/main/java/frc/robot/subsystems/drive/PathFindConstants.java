package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class PathFindConstants {

  // Offsets in meters (x = forward, y = left from robot POV)
  public static final Transform2d FrontOffset =
      new Transform2d(-0.25, 0.0, Rotation2d.fromDegrees(0.0));
  public static final Transform2d FrontRightOffset =
      new Transform2d(-0.25, -0.03, Rotation2d.fromDegrees(0.0));
  public static final Transform2d FrontLeftOffset =
      new Transform2d(-0.25, 0.03, Rotation2d.fromDegrees(0.0));

  public static final Transform2d BackOffset =
      new Transform2d(-0.25, 0.0, Rotation2d.fromDegrees(0.0));
  public static final Transform2d BackRightOffset =
      new Transform2d(-0.25, -0.03, Rotation2d.fromDegrees(0.0));
  public static final Transform2d BackLeftOffset =
      new Transform2d(-0.25, 0.03, Rotation2d.fromDegrees(0.0));

  public static final Transform2d PlayerStationLeftOffset =
      new Transform2d(0.25, 0.0, Rotation2d.fromDegrees(0.0));
  public static final Transform2d PlayerStationRightOffset =
      new Transform2d(0.25, 0.0, Rotation2d.fromDegrees(0.0));

  // Reef Positions (adjusted poses + proper offsets)
  public static final Pose2d A =
      new Pose2d(3.19, 4.25, Rotation2d.fromDegrees(0)).plus(FrontOffset);
  public static final Pose2d B =
      new Pose2d(3.19, 3.81, Rotation2d.fromDegrees(0)).plus(FrontOffset);
  public static final Pose2d C =
      new Pose2d(3.64, 2.97, Rotation2d.fromDegrees(60)).plus(FrontRightOffset);
  public static final Pose2d D =
      new Pose2d(4.01, 2.77, Rotation2d.fromDegrees(60)).plus(FrontRightOffset);
  public static final Pose2d E =
      new Pose2d(4.93, 2.75, Rotation2d.fromDegrees(120)).plus(BackRightOffset);
  public static final Pose2d F =
      new Pose2d(5.27, 2.97, Rotation2d.fromDegrees(120)).plus(BackRightOffset);
  public static final Pose2d G =
      new Pose2d(5.79, 3.81, Rotation2d.fromDegrees(180)).plus(BackOffset);
  public static final Pose2d H =
      new Pose2d(5.79, 4.25, Rotation2d.fromDegrees(180)).plus(BackOffset);
  public static final Pose2d I =
      new Pose2d(5.27, 5.05, Rotation2d.fromDegrees(-120)).plus(BackLeftOffset);
  public static final Pose2d J =
      new Pose2d(4.93, 5.30, Rotation2d.fromDegrees(-120)).plus(BackLeftOffset);
  public static final Pose2d K =
      new Pose2d(4.01, 5.28, Rotation2d.fromDegrees(-60)).plus(FrontLeftOffset);
  public static final Pose2d L =
      new Pose2d(3.64, 5.02, Rotation2d.fromDegrees(-60)).plus(FrontLeftOffset);

  // Player Station Positions (adjusted poses + proper offsets)
  public static final Pose2d PSRight =
      new Pose2d(1.19, 7.05, Rotation2d.fromDegrees(-55)).plus(PlayerStationRightOffset);
  public static final Pose2d PSLeft =
      new Pose2d(1.19, 1.0, Rotation2d.fromDegrees(55)).plus(PlayerStationLeftOffset);

  // PID Constants (may need tuning based on testing)
  public static final double kP = 8.0;
  public static final double kI = 0.325;
  public static final double kD = 0.0;

  // Max speed constraints (rotational in m/s for tank drive)
  public static final double maxRotationSpeed = 1.5;
}
