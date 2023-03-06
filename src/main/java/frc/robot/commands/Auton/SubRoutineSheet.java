package frc.robot.commands.Auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;
import frc.robot.commands.Routines.BasicMovement.Wait;

public class SubRoutineSheet {
    public static ParallelDeadlineGroup substationIntake;

    public static void initSubRoutines() {
        substationIntake = new ParallelDeadlineGroup(
            new UseClaw(),
            new TurnAndTranslate(new Rotation2d(0), new Rotation2d(0), 1),
            new SequentialCommandGroup(
                new Wait(0.1),
                new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE)
            )
        );
    }
}
