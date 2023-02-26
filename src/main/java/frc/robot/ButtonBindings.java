package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.Constants.RoutineConstants.POSITION_TYPE;
import frc.robot.commands.Routines.ClawState;
import frc.robot.commands.Routines.MoveBotTo;
// import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RampPusher;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class ButtonBindings {

    private static SwerveDrive swerve;
    private static RampPusher pusher;
    private static Arm arm;
    private static Elevator elevator;
    private static Claw claw;

    public ButtonBindings(CommandJoystick joy, CommandJoystick operatorJoy1) {
        getSubsystems();

        joy.button(JoystickConstants.RESET_TELEMETRY).onTrue(new InstantCommand(() -> {
            swerve.setHeading(0);
            Pose2d resetPose = MiscConstants.DEBUG_RESET_POSE;
            swerve.resetOdometry(OdometryMath2023.isBlue() ? resetPose : OdometryMath2023.flip(resetPose));
            swerve.resetMods();
        }, swerve));

        joy.button(JoystickConstants.TOGGLE_FIELD_OP).onTrue(new InstantCommand(() -> {
            swerve.toggleFieldRelative();
        }));

        joy.button(JoystickConstants.TOGGLE_RAMP_PUSHER).onTrue(new InstantCommand(() -> {
            pusher.deploy(!pusher.isDeployed());
        }, pusher));

        joy.button(JoystickConstants.LOCK_SWERVE).onTrue(new InstantCommand(() -> {
            swerve.setLocked(!swerve.getLocked());
        }, swerve));

        // operatorJoy1.button(JoystickConstants.CANCEL_ALL_COMMANDS_O).onTrue(new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.HIGH_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.HIGH_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.MID_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CUBE_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.SUBSTATION_PRESET).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CUBES).onTrue(new MoveClawTo(RoutineConstants.CUBE_INTAKE_CLAW_STATE));
        // operatorJoy1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CONES).onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> {
        //     claw.grab(true);
        // }, claw),    
        // new MoveClawTo(RoutineConstants.CONE_INTAKE_CLAW_STATE)));

        operatorJoy1.button(JoystickConstants.debugMovement).onTrue(new MoveBotTo(RoutineConstants.debug));
    }

    private static void getSubsystems() {
        swerve = RobotContainer.getSwerve();
        pusher = RobotContainer.getRampPusher();
        // arm = RobotContainer.getArm();
        // elevator = RobotContainer.getElevator();
        // claw = RobotContainer.getClaw();
    }
}