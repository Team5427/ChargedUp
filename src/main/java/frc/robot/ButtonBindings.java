package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.RoutineConstants.POSITION_TYPE;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.PartyMode;
import frc.robot.commands.PushRamp;
import frc.robot.commands.UseClaw;
import frc.robot.commands.UseIntake;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.Balancing.BalanceLinear;
import frc.robot.commands.Routines.BasicMovement.TiltWheels;
import frc.robot.commands.Routines.BasicMovement.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class ButtonBindings {

    private static SwerveDrive swerve;
    private static Claw claw;
    private static Led led;
    private static Intake intake;

    public ButtonBindings(CommandJoystick operatorJoy1, CommandJoystick operatorJoy2, CommandXboxController xboxController) {
        getSubsystems();

        xboxController.y().onTrue(new InstantCommand(() -> {
            swerve.setHeadingRad(0);
            Translation2d resetTrans = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT.getTranslation();

            if (!OdometryMath2023.isBlue()) {
                resetTrans = OdometryMath2023.flip(resetTrans);
            }
            
            swerve.resetOdometry(new Pose2d(resetTrans, swerve.getRotation2d()));
            swerve.resetMods();
        }, swerve));

        xboxController.x().onTrue(new InstantCommand(() -> {
            swerve.toggleFieldRelative();
        }));

        xboxController.a().onTrue(new TiltWheels(SwerveConstants.X_WHEEL_ANGLES));
        xboxController.a().toggleOnTrue(new PartyMode());

        xboxController.b().onFalse(new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE));

        xboxController.start().onTrue(new InstantCommand(() -> {
            claw.toggleGrabber();
        }, claw));

        operatorJoy1.button(JoystickConstants.CANCEL_ALL_COMMANDS_O).onTrue(new ParallelCommandGroup(
            new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE),
            new InstantCommand(() -> {
                intake.setDeployed(false);
            })
        ));
        operatorJoy1.button(JoystickConstants.HIGH_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.MID_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.HIGH_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.MID_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CUBE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.FLIP_COLOR).onTrue(new InstantCommand(() -> {
            led.togglePurple();
            led.setState(Led.INTAKE);
            intake.setDeployed(false);
        }));
        operatorJoy1.button(JoystickConstants.SUBSTATION_PRESET).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CUBES).onTrue(
            new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE)
        );
        operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CONES).onTrue(
            new MoveClawTo(RoutineConstants.CONE_INTAKE_CLAW_STATE)
        );       
        operatorJoy2.button(JoystickConstants.BALANCE_BTN).onTrue(new BalanceLinear());

        // operatorJoy1.button(JoystickConstants.ARM_RESET).whileTrue(new ManualArm(ArmConstants.MANUAL_ARM_SPEED));

        operatorJoy2.button(JoystickConstants.FLOOR_INTAKE_LED).onTrue(new InstantCommand(() -> {
            led.setState(Led.INTAKE_FLOOR);
        }));

        operatorJoy2.button(JoystickConstants.OPERATOR_USE_END_EFF).onTrue(new InstantCommand(() -> {
            if(led.getState() == Led.INTAKE_FLOOR || led.getState() == Led.SCORING_FLOOR){
                CommandScheduler.getInstance().schedule(new UseIntake());
            } else {
                CommandScheduler.getInstance().schedule(new UseClaw());
            }
        }));

        operatorJoy2.button(JoystickConstants.RAMP_PUSH).whileTrue(new PushRamp(false, 0.0));

        operatorJoy1.button(JoystickConstants.OPERATOR_SUBSTATION).onTrue(
            new ParallelCommandGroup(
                new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        claw.grab(false);
                    }),
                    new Wait(0.25),
                    new UseClaw()
                )
            )
        );

        operatorJoy2.button(JoystickConstants.TOP_LEFT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.LEFT_CONE),
            new SequentialCommandGroup(
                new Wait(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE)
            )
        ));
        operatorJoy2.button(JoystickConstants.TOP_RIGHT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.RIGHT_CONE),
            new SequentialCommandGroup(
                new Wait(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE)
            )
        ));
        operatorJoy2.button(JoystickConstants.MID_LEFT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.LEFT_CONE),
            new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE)
        ));

        operatorJoy2.button(JoystickConstants.MID_RIGHT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.RIGHT_CONE),
            new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE)
        ));
    }

    private static void getSubsystems() {
        swerve = RobotContainer.getSwerve();
        claw = RobotContainer.getClaw();
        led = RobotContainer.getLed();
        intake = RobotContainer.getIntake();
    }


}