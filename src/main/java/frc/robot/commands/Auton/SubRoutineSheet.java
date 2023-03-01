package frc.robot.commands.Auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.ClawState;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.TurnAndTranslate;
import frc.robot.commands.Routines.BasicMovement.MoveRobotOriented;
import frc.robot.util.OdometryMath2023;

public class SubRoutineSheet {
    public static ParallelDeadlineGroup substationIntake;
    public static SequentialCommandGroup stationBalance;

    public static void initSubRoutines() {
        if (OdometryMath2023.isBlue()) {
            substationIntake = new ParallelDeadlineGroup(
                new UseClaw(),
                new TurnAndTranslate(0.0, 0.0, .5, 1),
                new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE)
            );
        } else {
            substationIntake = new ParallelDeadlineGroup(
                new UseClaw(),
                new TurnAndTranslate(Math.PI, Math.PI, .5, 1),
                new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE)
            );
        }

        stationBalance = new SequentialCommandGroup(
            new MoveRobotOriented(1.25, 2, new Rotation2d(0)),
            new Balance(),
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setLocked(true);
            }, RobotContainer.getSwerve())
        );
    }
}
