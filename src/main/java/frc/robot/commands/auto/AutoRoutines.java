package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.util.LoggedAutoChooser;

public class AutoRoutines {
    private final AutoFactory autoFactory;
    private final Drive drive;

    public AutoRoutines(Drive drive, Roller roller, LoggedAutoChooser autoChooser) {
        this.drive = drive;

        autoFactory = drive.createAutoFactory((traj, isStart) -> {
            Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
            Logger.recordOutput("Odometry/IsStart", isStart);
        });

        autoFactory.bind("Eject", roller.runPercent(0.5).withTimeout(1));

        autoChooser.addRoutine("4-piece L1", this::get4Piece);

        autoChooser.addRoutine("Center 1-piece", this::getCenter1Piece);

        // double correctionRemoveMeWhenItActuallyWorks = 4;
        // double distanceInches = 90 - 3 - 1 + correctionRemoveMeWhenItActuallyWorks;
        // double timeSeconds = 1.5;
        // autoChooser.addCmd("1-piece center", () -> Commands.sequence(
        //     DriveCommands.driveStraightCommand(drive, Units.inchesToMeters(distanceInches / timeSeconds),
        //         RobotState.getInstance()::getRotation).withTimeout(timeSeconds),
        //     roller.runPercent(0.5).withTimeout(1.5)));
    }

    private AutoRoutine get4Piece() {
        var routine = autoFactory.newRoutine("4-piece L1");

        AutoTrajectory firstPiece = routine.trajectory("4-piece L1", 0);
        AutoTrajectory secondPiece = routine.trajectory("4-piece L1", 1);
        AutoTrajectory thirdPiece = routine.trajectory("4-piece L1", 2);
        // AutoTrajectory fourthPiece = routine.trajectory("4-piece L1", 3);

        // @formatter:off
        routine.active().onTrue(Commands.sequence(
            firstPiece.resetOdometry(),
            firstPiece.cmd(),
            Commands.runOnce(drive::stop),
            Commands.waitSeconds(2.0),
            secondPiece.cmd(),
            Commands.runOnce(drive::stop),
            Commands.waitSeconds(2.0),
            thirdPiece.cmd(),
            Commands.runOnce(drive::stop)
        ));
        // @formatter:on

        return routine;
    }

    private AutoRoutine getCenter1Piece() {
        var routine = autoFactory.newRoutine("Center 1-piece");

        AutoTrajectory firstPiece = routine.trajectory("Center 1-piece", 0);

        // @formatter:off
        routine.active().onTrue(Commands.sequence(
            firstPiece.resetOdometry(),
            firstPiece.cmd(),
            Commands.runOnce(drive::stop)
        ));
        // @formatter:on

        return routine;
    }
}
