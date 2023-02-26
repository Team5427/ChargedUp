package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RampPusher;

public class RampPusherDebug extends CommandBase {

    CommandJoystick joy;
    RampPusher rampPusher;

    public RampPusherDebug() {
        rampPusher = RobotContainer.getRampPusher();
        joy = RobotContainer.getJoy();
        addRequirements(rampPusher);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if (joy.button(JoystickConstants.debugRampUp).getAsBoolean()
        // //  && 
        // // (rampPusher.getPosition() < -2.5)
        // ) {
        //     rampPusher.move(.35);
        // } else if (joy.button(JoystickConstants.debugRampDown).getAsBoolean()
        // //  && (rampPusher.getPosition() > 0)
        //  ) {
        //     rampPusher.move(-.35);
        // } else {
        //     rampPusher.stop();
        // }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
