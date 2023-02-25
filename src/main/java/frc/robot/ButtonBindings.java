package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.Constants.RoutineConstants.POSITION_TYPE;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RampPusher;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class ButtonBindings {

    private static SwerveDrive swerve;
    private static RampPusher pusher;
    private static Arm arm;
    private static Elevator elevator;

    public ButtonBindings(CommandJoystick pilot, CommandJoystick operator1, CommandJoystick operator2) {
        getSubsystems();

        pilot.button(JoystickConstants.RESET_TELEMETRY).onTrue(new InstantCommand(() -> {
            swerve.setHeading(0);
            Pose2d resetPose = MiscConstants.DEBUG_RESET_POSE;
            swerve.resetOdometry(OdometryMath2023.isBlue() ? resetPose : OdometryMath2023.flip(resetPose));
            swerve.resetMods();
        }));

        pilot.button(JoystickConstants.TOGGLE_FIELD_OP).onTrue(new InstantCommand(() -> {
            swerve.toggleFieldRelative();
        }));

        pilot.button(JoystickConstants.TOGGLE_RAMP_PUSHER).onTrue(new InstantCommand(() -> {
            pusher.deploy(!pusher.isDeployed());
        }, pusher));

        pilot.button(JoystickConstants.LOCK_SWERVE_P).onTrue(new InstantCommand(() -> {
            swerve.setLocked(swerve.getLocked());
        }, swerve));

        operator1.button(JoystickConstants.CANCEL_ALL_COMMANDS_O).onTrue(new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE));
        operator1.button(JoystickConstants.HIGH_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CONE_CLAW_STATE));
        operator1.button(JoystickConstants.MID_CONE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CONE_CLAW_STATE));
        operator1.button(JoystickConstants.HIGH_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.TOP_CUBE_CLAW_STATE));
        operator1.button(JoystickConstants.MID_CUBE_PRESET).onTrue(new MoveClawTo(RoutineConstants.MID_CUBE_CLAW_STATE));
        operator1.button(JoystickConstants.LOW_SCORE_PRESET).onTrue(new MoveClawTo(RoutineConstants.LOW_CLAW_STATE));
        operator1.button(JoystickConstants.SUBSTATION_PRESET).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
        operator1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CUBES).onTrue(new MoveClawTo(RoutineConstants.CUBE_FLOOR_INTAKE_CLAW_STATE));
        operator1.button(JoystickConstants.FLOOR_INTAKE_PRESET_CONES).onTrue(new MoveClawTo(RoutineConstants.CONE_FLOOR_INTAKE_CLAW_STATE));

        operator2.button(JoystickConstants.LOCK_SWERVE_O).onTrue(new InstantCommand(() -> {
            swerve.setLocked(true);
        }, swerve));

        operator2.button(JoystickConstants.UNLOCK_SWERVE_O).onTrue(new InstantCommand(() -> {
            swerve.setLocked(false);
        }, swerve));

        operator2.button(JoystickConstants.LEFT_CONE).onTrue(new MoveBotTo(POSITION_TYPE.LEFT_CONE));
        operator2.button(JoystickConstants.CUBE).onTrue(new MoveBotTo(POSITION_TYPE.CUBE));
        operator2.button(JoystickConstants.RIGHT_CONE).onTrue(new MoveBotTo(POSITION_TYPE.RIGHT_CONE));
        operator2.button(JoystickConstants.LEFT_SS).onTrue(new MoveBotTo(POSITION_TYPE.LEFT_SS));
        operator2.button(JoystickConstants.RIGHT_SS).onTrue(new MoveBotTo(POSITION_TYPE.RIGHT_SS));
        operator2.button(JoystickConstants.debugButton).onTrue(new MoveBotTo(RoutineConstants.debugPreset));
    }

    private static void getSubsystems() {
        swerve = RobotContainer.getSwerve();
        pusher = RobotContainer.getRampPusher();
        arm = RobotContainer.getArm();
        elevator = RobotContainer.getElevator();
    }
}