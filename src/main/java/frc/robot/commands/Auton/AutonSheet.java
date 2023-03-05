package frc.robot.commands.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.Balance;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.BasicMovement.MoveRobotOriented;
import frc.robot.pathUtil.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup testAuton;
    private static Command swervePath1;
    public static SequentialCommandGroup stationBalance;

    public static void initAutons() {
        swervePath1 = SwervePathMaker.getCommand("StraightLinePath");

        stationBalance = new SequentialCommandGroup(
            new MoveRobotOriented(2, 1, new Rotation2d(0)),
            new Balance(),
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setLocked(true);
            }, RobotContainer.getSwerve())
        );

        testAuton = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
            swervePath1,
            stationBalance
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

    }
}
