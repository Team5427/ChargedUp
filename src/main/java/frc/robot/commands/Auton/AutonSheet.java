package frc.robot.commands.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RoutineConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.StationBalance;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;
import frc.robot.pathUtil.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup bottomSingleConeEngage;
    private static Command bottomSingleConeEngagePath1;
    public static SequentialCommandGroup topSingleConeEngage;
    private static Command topSingleConeEngagePath1;
    public static SequentialCommandGroup chargeStationThroughAndBack;

    public static void initAutons() {
        bottomSingleConeEngagePath1 = SwervePathMaker.getCommand("BottomSingleConeEngage1");
        topSingleConeEngagePath1 = SwervePathMaker.getCommand("TopSingleConeEngage1");

        bottomSingleConeEngage = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                bottomSingleConeEngagePath1    
            ),
            new StationBalance(true, false)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        topSingleConeEngage = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                topSingleConeEngagePath1    
            ),
            new StationBalance(true, true)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        chargeStationThroughAndBack = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new TurnAndTranslate(new Rotation2d(Math.PI), new Rotation2d(0), 3, 4.0),
            new StationBalance(true, true)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });
    }
}
