package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private DriveCommands() {
    }

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero)).getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        var robotState = RobotState.getInstance();
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.maxSpeedMetersPerSec,
                linearVelocity.getY() * DriveConstants.maxSpeedMetersPerSec,
                omega * DriveConstants.maxAngularSpeedRadPerSec);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? robotState.getRotation().plus(new Rotation2d(Math.PI)) : robotState.getRotation()));
        }, drive);
    }

    private static final LoggedTunableNumber lineupDistance = new LoggedTunableNumber("AutoScore/ReefLineupDistance",
        5);

    private static final LoggedTunableNumber centerDistanceTweak = new LoggedTunableNumber( //
        "AutoScore/CenterDistanceTweak", 0.0);
    private static final LoggedTunableNumber centerDistanceTweakRight = new LoggedTunableNumber( //
        "AutoScore/CenterPositionTweakRight", -0.25);

    public static Pose2d getLineupPose(ReefTarget target) {
        return getLineupPose(target, 0.);
    }

    private static Pose2d getLineupPose(ReefTarget target, double offsetOutwardInches) {
        double distanceAwayInches = lineupDistance.get();
        distanceAwayInches += offsetOutwardInches;

        double centerDistance = FieldConstants.reefBranchSeparation.in(Meters) / 2.
            + Units.inchesToMeters(centerDistanceTweak.get());
        boolean isLeft = target.branch().isLeft;
        double horizontalOffset = (isLeft ? -centerDistance : centerDistance);

        Transform2d tagRelativeOffset = new Transform2d(new Translation2d(Units.inchesToMeters(distanceAwayInches),
            horizontalOffset + Units.inchesToMeters(centerDistanceTweakRight.get())), Rotation2d.k180deg);
        Pose2d fieldPose = target.branch().face.getTagPose().transformBy(tagRelativeOffset);

        return fieldPose;
    }

    public static Command PathfindtoBranch(Drive drive) {
        DriverStationInterface.getInstance().setReefTarget(
            new ReefTarget(DriverStationInterface.getInstance().getReefTarget().branch(), FieldConstants.ReefLevel.L1));
        Pose2d targetPose = getLineupPose(
            new ReefTarget(DriverStationInterface.getInstance().getReefTarget().branch(), FieldConstants.ReefLevel.L1));

        Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "Target Pose",
            targetPose.toString()));

        PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 6.9,
            (DriveConstants.maxSpeedMetersPerSec / 2.0), 1446);

        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);

        return Commands.sequence(pathfindingCommand);
    }
}
