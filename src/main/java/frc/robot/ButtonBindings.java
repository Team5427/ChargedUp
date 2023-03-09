package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.RoutineConstants.POSITION_TYPE;
import frc.robot.commands.ManualArm;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Auton.SubRoutineSheet;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.Balancing.BalanceLinear;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class ButtonBindings {

    private static SwerveDrive swerve;
    private static Arm arm;
    private static Elevator elevator;
    private static Claw claw;
    private static Led led;

    public ButtonBindings(CommandJoystick joy, CommandJoystick operatorJoy1, CommandJoystick operatorJoy2) {
        getSubsystems();

        joy.button(JoystickConstants.RESET_TELEMETRY).onTrue(new InstantCommand(() -> {
            swerve.setHeadingRad(0);
            Translation2d resetTrans = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT.getTranslation();

            if (!OdometryMath2023.isBlue()) {
                resetTrans = OdometryMath2023.flip(resetTrans);
            }
            
            swerve.resetOdometry(new Pose2d(resetTrans, swerve.getRotation2d()));
            swerve.resetMods();
        }, swerve));

        joy.button(JoystickConstants.TOGGLE_FIELD_OP).onTrue(new InstantCommand(() -> {
            swerve.toggleFieldRelative();
        }));

        joy.button(JoystickConstants.LOCK_SWERVE).onTrue(new InstantCommand(() -> {
            swerve.setLocked(!swerve.getLocked());
        }, swerve));

        joy.button(JoystickConstants.CLAW_BTN).onTrue(new UseClaw());
        joy.button(JoystickConstants.SS).onTrue(SubRoutineSheet.substationIntake);

        joy.button(JoystickConstants.CLAW_INTAKE).whileTrue(new ManualClaw(ClawConstants.INTAKE_SPEED_DECIMAL));
        joy.button(JoystickConstants.CLAW_OUTTAKE).whileTrue(new ManualClaw(ClawConstants.OUTTAKE_SPEED_DECIMAL));
        joy.button(JoystickConstants.CLAW_CLAMP).onTrue(new InstantCommand(() ->{
            claw.toggleGrabber();
        }, claw));

        joy.button(JoystickConstants.SS_CANCEL).onTrue(new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE));


        operatorJoy1.button(JoystickConstants.CANCEL_ALL_COMMANDS_O).onTrue(new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.HIGH_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.MID_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.HIGH_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.MID_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CUBE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.FLIP_COLOR).onTrue(new InstantCommand(() -> {
            led.togglePurple();
            led.setState(led.INTAKE);
        }));
        operatorJoy1.button(JoystickConstants.SUBSTATION_PRESET).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CUBES).onTrue(new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE));
        operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CONES).onTrue(new MoveClawTo(RoutineConstants.CONE_INTAKE_CLAW_STATE));
        operatorJoy2.button(JoystickConstants.BALANCE_BTN).onTrue(new BalanceLinear());

        operatorJoy1.button(JoystickConstants.ARM_RESET).whileTrue(new ManualArm(ArmConstants.MANUAL_ARM_SPEED));

        operatorJoy2.button(JoystickConstants.TOP_LEFT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.LEFT_CONE),
            new SequentialCommandGroup(
                new WaitCommand(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE)
            )
        ));
        operatorJoy2.button(JoystickConstants.TOP_RIGHT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.RIGHT_CONE),
            new SequentialCommandGroup(
                new WaitCommand(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE)
            )
        ));
        operatorJoy2.button(JoystickConstants.MID_LEFT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.LEFT_CONE),
            new SequentialCommandGroup(
                new WaitCommand(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE)
            )
        ));

        operatorJoy2.button(JoystickConstants.MID_RIGHT_SCORE).onTrue(new ParallelCommandGroup(
            new MoveBotTo(POSITION_TYPE.RIGHT_CONE),
            new SequentialCommandGroup(
                new WaitCommand(RoutineConstants.DEBUG_INTEGRATE_DELAY_TIME),
                new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE)
            )
        ));

    
    }

    private static void getSubsystems() {
        swerve = RobotContainer.getSwerve();
        arm = RobotContainer.getArm();
        elevator = RobotContainer.getElevator();
        claw = RobotContainer.getClaw();
        led = RobotContainer.getLed();
    }


}