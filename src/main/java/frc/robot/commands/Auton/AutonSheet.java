
package frc.robot.commands.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.PushRamp;
import frc.robot.commands.UseClaw;
import frc.robot.commands.UseIntake;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.Balancing.BalanceLinear;
import frc.robot.commands.Routines.BasicMovement.TiltWheels;
import frc.robot.commands.Routines.BasicMovement.TimedChassisMove;
import frc.robot.commands.Routines.BasicMovement.TurnInPlace;
import frc.robot.commands.Routines.BasicMovement.Wait;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Led;
import frc.robot.util.OdometryMath2023;

public class AutonSheet {
    public static SequentialCommandGroup topDoubleConeScore;
    private static Command topDoubleConeScore1;
    private static Command topDoubleConeScore2;
    public static SequentialCommandGroup floorTripleBumpSide;
    private static Command floorTripleBumpSide1;
    private static Command floorTripleBumpSide2;
    public static SequentialCommandGroup scoreBalanceAuto;
    public static SequentialCommandGroup turninplace;

    public static void initAutons() {
        topDoubleConeScore1 = SwervePathMaker.getCommand("TopDoubleConeScore1");
        topDoubleConeScore2 = SwervePathMaker.getCommand("TopDoubleConeScore2");
        floorTripleBumpSide1 = SwervePathMaker.getCommand("FloorTripleBumpSide1");
        floorTripleBumpSide2 = SwervePathMaker.getCommand("FloorTripleBumpSide2");
    
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
                        new WaitCommand(5)
                    ),
                    new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE),
                    new InstantCommand(() -> {
                        System.out.println("claw moved to scoring " + Timer.getFPGATimestamp());
                    })
                )
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> {
                    // System.out.println("doing thing " + Timer.getFPGATimestamp());
                    RobotContainer.getClaw().set(ClawConstants.OUTTAKE_SPEED_DECIMAL);
                }, RobotContainer.getClaw()),
                new Wait(ClawConstants.CUBE_OUTTAKE_EXCESS_TIME_S + 0.2)
            ),
            new ParallelCommandGroup(
                new ParallelCommandGroup(
                    new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                    new InstantCommand(()->{
                        RobotContainer.getLed().setPurple(false);
                    })
                ),
                topDoubleConeScore2
            )
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

        scoreBalanceAuto = new SequentialCommandGroup(
            new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE),
            new UseClaw(),
            new ParallelCommandGroup(
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
                new SequentialCommandGroup(
                    new TimedChassisMove(new Translation2d(-0.75, 0), 0.125),
                    new TurnInPlace(new Rotation2d(0)),
                    new ParallelCommandGroup(
                        new PushRamp(true, 1.75),
                        new TimedChassisMove(new Translation2d(3.5, 0), 1.5)
                    ),
                    new TimedChassisMove(new Translation2d(0.75, 0), 3.5),
                    new TurnInPlace(new Rotation2d(Math.PI)),
                    new Wait(0.25),
                    new ParallelCommandGroup(
                        new TimedChassisMove(new Translation2d(3.5, 0), 1.5),
                        new PushRamp(true, 3)
                    )
                )
            ),
            new BalanceLinear(),
            new TiltWheels(SwerveConstants.X_WHEEL_ANGLES)
        );

        turninplace = new SequentialCommandGroup(
            new TurnInPlace(new Rotation2d(Math.PI))
        );
    }
}
