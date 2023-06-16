
package frc.robot.commands.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.UseClaw;
import frc.robot.commands.UseIntake;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.Balancing.BalanceDoubleP;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;
import frc.robot.commands.Routines.BasicMovement.Wait;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Led;
import frc.robot.util.OdometryMath2023;

public class AutonSheet {
    public static SequentialCommandGroup topSingleConeIntakeEngage;
    private static Command topSingleConeIntakeEngage1;
    public static SequentialCommandGroup topSingleConeIntake;
    public static SequentialCommandGroup topDoubleConeScore;
    private static Command topDoubleConeScore1;
    private static Command topDoubleConeScore2;
    public static SequentialCommandGroup bottomSingleConeIntakeEngage;
    private static Command bottomSingleConeIntakeEngage1;
    public static SequentialCommandGroup topDoubleConeEngage;
    private static Command topDoubleConeEngage1;
    private static Command topDoubleConeEngage2;
    public static SequentialCommandGroup floorTripleCleanSide;
    private static Command floorTripleCleanSide1;
    private static Command floorTripleCleanSide2;
    public static SequentialCommandGroup floorTripleBumpSide;
    private static Command floorTripleBumpSide1;
    private static Command floorTripleBumpSide2;
    public static SequentialCommandGroup robonautsAuton;
    public static SequentialCommandGroup integratedAuton;
    private static Command integratedAuton1;
    private static Command integratedAuton2;

    public static void initAutons() {
        topSingleConeIntakeEngage1 = SwervePathMaker.getCommand("TopSingleConeIntakeEngage1");
        topDoubleConeScore1 = SwervePathMaker.getCommand("TopDoubleConeScore1");
        topDoubleConeScore2 = SwervePathMaker.getCommand("TopDoubleConeScore2");
        bottomSingleConeIntakeEngage1 = SwervePathMaker.getCommand("BottomSingleConeIntakeEngage1");
        topDoubleConeEngage1 = SwervePathMaker.getCommand("TopDoubleConeEngage1");
        topDoubleConeEngage2 = SwervePathMaker.getCommand("TopDoubleConeEngage2");
        floorTripleCleanSide1 = SwervePathMaker.getCommand("FloorTripleCleanSide1");
        floorTripleCleanSide2 = SwervePathMaker.getCommand("FloorTripleCleanSide2");
        floorTripleBumpSide1 = SwervePathMaker.getCommand("FloorTripleBumpSide1");
        floorTripleBumpSide2 = SwervePathMaker.getCommand("FloorTripleBumpSide2");
        integratedAuton1 = SwervePathMaker.getCommand("IntegratedAuton1");
        integratedAuton2 = SwervePathMaker.getCommand("IntegratedAuton2");
    
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
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)
        );

        topDoubleConeScore = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    topDoubleConeScore1,
                    new InstantCommand(() -> {
                        System.out.println("path 1 finished " + Timer.getFPGATimestamp());
                    })
                ),
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
                    new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE),
                    new InstantCommand(() -> {
                        System.out.println("claw moved to scoring " + Timer.getFPGATimestamp());
                    })
                )
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> {
                    System.out.println("doing thing " + Timer.getFPGATimestamp());
                    RobotContainer.getClaw().set(ClawConstants.OUTTAKE_SPEED_DECIMAL);
                }, RobotContainer.getClaw()),
                new Wait(ClawConstants.CUBE_OUTTAKE_EXCESS_TIME_S + 0.2)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE),
                        new InstantCommand(()->{
                            RobotContainer.getLed().setPurple(true);
                        }),
                        new ParallelRaceGroup(
                            new SequentialCommandGroup(
                                new WaitCommand(.5),
                                new UseClaw()        
                            ),
                            new WaitCommand(4)
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
            )
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
                                new WaitCommand(.25),
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
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                topDoubleConeEngage2
            ),
            new BalanceDoubleP()
        );

        floorTripleBumpSide = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getLed().setState(Led.INTAKE_FLOOR);
            }),
            new WaitCommand(0.3),
            new ParallelCommandGroup(
                floorTripleBumpSide1,
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new UseIntake() //INTAKES CUBE 1
                    ),
                    new WaitCommand(5)
                )
            ),
            new InstantCommand(() -> {
                OdometryMath2023.reseedOdometry();
            }),
            new ParallelCommandGroup(
                floorTripleBumpSide2,

                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new UseIntake(), //OUTTAKES CUBE 1
                        new WaitCommand(1.5),
                        new UseIntake() //INTAKES CUBE 2
                    ),
                    new WaitCommand(5) //FIXME    
                )
            )
        );

        robonautsAuton = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
            new WaitCommand(1), //Wait for arm to stow
            new TurnAndTranslate(new Rotation2d(3 * Math.PI/4), new Rotation2d(0), 2.5, 2), //RAMS ONTO CHARGE STATION
            new TurnAndTranslate(new Rotation2d(3 * Math.PI/4), new Rotation2d(0), 1, 3), //SLOWS DOWN AND GETS OFF
            new TurnAndTranslate(new Rotation2d(3 * Math.PI/4), new Rotation2d(Math.PI), 2.5, 2), //RAMS BACK ON THE OTHER WAY
            new BalanceDoubleP()
        );

        integratedAuton = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                integratedAuton1,
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
            new UseClaw(),
            new InstantCommand(() -> {
                RobotContainer.getLed().setState(Led.INTAKE_FLOOR);
                Robot.specialAuton = true;
            }),
            new ParallelCommandGroup(
                integratedAuton2,
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new UseIntake()
                )
            )
        );
    }
}
