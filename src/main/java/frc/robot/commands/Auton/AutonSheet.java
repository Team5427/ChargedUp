package frc.robot.commands.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RoutineConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.StationBalance;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;
import frc.robot.pathUtil.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup bottomSingleConeEngage;
    private static Command bottomSingleConeEngage1;
    public static SequentialCommandGroup topSingleConeEngage;
    private static Command topSingleConeEngage1;
    public static SequentialCommandGroup chargeStationThroughAndBack;
    public static SequentialCommandGroup topSingleConeIntakeEngage;
    private static Command topSingleConeIntakeEngage1;
    public static SequentialCommandGroup bottomSingleCone;
    private static Command bottomSingleCone1;
    public static SequentialCommandGroup topDoubleConeEngage;
    private static Command topDoubleConeEngage1;
    private static Command topDoubleConeEngage2;
    public static SequentialCommandGroup topSingleConeIntake;
    private static Command topSingleConeIntake1;
    public static SequentialCommandGroup justScore;
    public static SequentialCommandGroup bottomSingleConeIntakeEngage;
    private static Command bottomSingleConeIntakeEngage1;

    public static void initAutons() {
        bottomSingleConeEngage1 = SwervePathMaker.getCommand("BottomSingleConeEngage1");
        topSingleConeEngage1 = SwervePathMaker.getCommand("TopSingleConeEngage1");
        topSingleConeIntakeEngage1 = SwervePathMaker.getCommand("TopSingleConeIntakeEngage1");
        bottomSingleCone1 = SwervePathMaker.getCommand("BottomSingleCone1");
        topDoubleConeEngage1 = SwervePathMaker.getCommand("TopDoubleConeEngage1");
        topDoubleConeEngage2 = SwervePathMaker.getCommand("TopDoubleConeEngage2");
        topSingleConeIntake1 = SwervePathMaker.getCommand("TopSingleConeIntake1");
        bottomSingleConeIntakeEngage1 = SwervePathMaker.getCommand("BottomSingleConeIntakeEngage1");
        

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
                bottomSingleConeEngage1    
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
                topSingleConeEngage1    
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
            new ParallelCommandGroup(
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                new TurnAndTranslate(new Rotation2d(Math.PI), new Rotation2d(0), 3, 2) //FIXME
            ),
            new TurnAndTranslate(new Rotation2d(Math.PI), new Rotation2d(0), 1.5, 3), //FIXME
            new StationBalance(true, true)    
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        topSingleConeIntakeEngage = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topSingleConeIntakeEngage1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new UseClaw()
                            ),
                            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)
                        ),
                        new WaitCommand(5)
                    )
                    ,
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
                )
            ),
            new StationBalance(true, false)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        bottomSingleConeIntakeEngage = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                bottomSingleConeIntakeEngage1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new UseClaw()
                            ),
                            new SequentialCommandGroup(
                                new WaitCommand(0.75),
                                new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)
                            )
                        ),
                        new WaitCommand(5)
                    )
                    ,
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
                )
            ),
            new StationBalance(true, false)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        bottomSingleCone = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                bottomSingleCone1,
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
            ),
            new InstantCommand(() -> {
                RobotContainer.getLed().setPurple(true);
            })
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        topDoubleConeEngage = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topDoubleConeEngage1,
                new SequentialCommandGroup(
                    new WaitCommand(0.50),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE),
                            new InstantCommand(() -> {
                                RobotContainer.getLed().setPurple(true);
                            }),
                            new UseClaw()
                        ),
                        new WaitCommand(5)
                    ),
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
                )
            ),
            new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topDoubleConeEngage2,
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
            ),
            new StationBalance(false, true)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        topSingleConeIntake = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topSingleConeIntake1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new UseClaw(),
                            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)                        
                        ),
                        new WaitCommand(5)
                    )
                )
            ),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });

        justScore = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().setHeadingRad(Math.PI);
                RobotContainer.getLed().setPurple(false);
            }),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });
    }
}
