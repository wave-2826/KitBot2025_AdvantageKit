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

    public AutoRoutines(Drive drive, Roller roller, LoggedAutoChooser autoChooser) {
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
            Commands.waitSeconds(1),
            secondPiece.cmd(),
            Commands.waitSeconds(1),
            thirdPiece.cmd(),
            Commands.waitSeconds(1),
            fourthPiece.cmd()
        ));
        // @formatter:on

        return routine;
    }
}
