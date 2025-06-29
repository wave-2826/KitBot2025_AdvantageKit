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

        autoFactory.bind("Eject", roller.runPercent(0.4).withTimeout(1));

        autoChooser.addRoutine("4-piece L1", this::get4Piece);
    }

    private AutoRoutine get4Piece() {
        var routine = autoFactory.newRoutine("4-piece L1");

        AutoTrajectory firstPiece = routine.trajectory("4-piece L1", 0);
        AutoTrajectory secondPiece = routine.trajectory("4-piece L1", 1);
        AutoTrajectory thirdPiece = routine.trajectory("4-piece L1", 2);
        AutoTrajectory fourthPiece = routine.trajectory("4-piece L1", 3);

        // @formatter:off
        routine.active().onTrue(Commands.sequence(
            firstPiece.resetOdometry(),
            firstPiece.cmd(),
            Commands.waitSeconds(0.25),
            Commands.runOnce(drive::stop),
            Commands.waitSeconds(0.75),
            secondPiece.cmd(),
            Commands.waitSeconds(0.25),
            Commands.runOnce(drive::stop),
            Commands.waitSeconds(0.75),
            thirdPiece.cmd(),
            Commands.waitSeconds(0.25),
            Commands.runOnce(drive::stop),
            Commands.waitSeconds(0.75),
            fourthPiece.cmd(),
            Commands.waitSeconds(0.25),
            Commands.runOnce(drive::stop)
        ));
        // @formatter:on

        return routine;
    }
}
