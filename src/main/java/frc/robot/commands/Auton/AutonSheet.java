
package frc.robot.commands.Auton;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RoutineConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.Balancing.BalanceDoubleP;
import frc.robot.pathUtil.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup topSingleConeIntakeEngage;
    private static Command topSingleConeIntakeEngage1;
    public static SequentialCommandGroup topSingleConeIntake;
    private static Command topSingleConeIntake1;
    public static SequentialCommandGroup topDoubleConeScore;
    private static Command topDoubleConeScore1;
    private static Command topDoubleConeScore2;
    public static SequentialCommandGroup bottomSingleConeIntakeEngage;
    private static Command bottomSingleConeIntakeEngage1;
    public static SequentialCommandGroup topDoubleConeEngage;
    private static Command topDoubleConeEngage1;
    private static Command topDoubleConeEngage2;

    public static void initAutons() {
        topSingleConeIntakeEngage1 = SwervePathMaker.getCommand("TopSingleConeIntakeEngage1");
        topSingleConeIntake1 = SwervePathMaker.getCommand("TopSingleConeIntake1");
        topDoubleConeScore1 = SwervePathMaker.getCommand("TopDoubleConeScore1");
        topDoubleConeScore2 = SwervePathMaker.getCommand("TopDoubleConeScore2");
        bottomSingleConeIntakeEngage1 = SwervePathMaker.getCommand("BottomSingleConeIntakeEngage1");
        topDoubleConeEngage1 = SwervePathMaker.getCommand("TopDoubleConeEngage1");
        topDoubleConeEngage2 = SwervePathMaker.getCommand("TopDoubleConeEngage2");
    
        topSingleConeIntakeEngage = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topSingleConeIntakeEngage1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new UseClaw()
                            ),
                            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)
                        ),
                        new WaitCommand(3.275)

                    )
                    ,
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
                )
            ),
            new BalanceDoubleP()
        );

        topSingleConeIntake = new SequentialCommandGroup(
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
                            new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new UseClaw()
                            ),
                            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)                        
                        ),
                        new WaitCommand(3.275)
                    ),
                    new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE)
                )
            ),
            new ParallelRaceGroup(
                new UseClaw(),
                new WaitCommand(1.5)
            ),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
        );

        topDoubleConeScore = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topDoubleConeScore1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.7),
                                new UseClaw()
                            ),
                            new SequentialCommandGroup(
                                new WaitCommand(.5),
                                new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE) 
                            )                      
                        ),
                        new WaitCommand(3.5)
                    ),
                    new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE)
                )
            ),
            new ParallelRaceGroup(
                new UseClaw(),
                new WaitCommand(1)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new MoveClawTo(RoutineConstants.CONE_INTAKE_CLAW_STATE),
                        new InstantCommand(()->{
                            RobotContainer.getLed().setPurple(false);
                        }),
                        new ParallelRaceGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(.5),
                                new UseClaw()        
                            ),
                            new WaitCommand(5)
                        )
                    ),
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)

                ),
                topDoubleConeScore2
            )
        );

        bottomSingleConeIntakeEngage = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                bottomSingleConeIntakeEngage1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new UseClaw()
                            ),
                            new SequentialCommandGroup(
                                new WaitCommand(1),
                                new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)
                            )
                        ),
                        new WaitCommand(4)

                    )
                    ,
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
                )
            ),
            new BalanceDoubleP()
        );
       
        topDoubleConeEngage = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                topDoubleConeEngage1,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        RobotContainer.getLed().setPurple(true);
                    }),
                    new ParallelRaceGroup(
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(0.7),
                                new UseClaw()
                            ),
                            new SequentialCommandGroup(
                                new WaitCommand(.5),
                                new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE) 
                            )                      
                        ),
                        new WaitCommand(3.5)
                    ),
                    new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE)
                )
            ),
            new ParallelRaceGroup(
                new UseClaw(),
                new WaitCommand(1)
            ),
            topDoubleConeEngage2,
            new BalanceDoubleP()
        );
    }
}
