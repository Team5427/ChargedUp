package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.subsystems.RampPusher;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class ButtonBindings {

    private static SwerveDrive swerve;
    private static RampPusher pusher;

    public ButtonBindings(CommandJoystick joy) {
        getSubsystems();

        joy.button(JoystickConstants.RESET_TELEMETRY).onTrue(new InstantCommand(() -> {
            swerve.setHeading(0);
            Pose2d resetPose = MiscConstants.DEBUG_RESET_POSE;
            swerve.resetOdometry(OdometryMath2023.isBlue() ? resetPose : OdometryMath2023.flip(resetPose));
            swerve.resetMods();
        }));

        joy.button(JoystickConstants.TOGGLE_FIELD_OP).onTrue(new InstantCommand(() -> {
            swerve.toggleFieldRelative();
        }));

        joy.button(JoystickConstants.TOGGLE_RAMP_PUSHER).onTrue(new InstantCommand(() -> {
            pusher.deploy(!pusher.isDeployed());
        }, pusher));    

    }

    private static void getSubsystems() {
        swerve = RobotContainer.getSwerve();
        pusher = RobotContainer.getRampPusher();
    }
}