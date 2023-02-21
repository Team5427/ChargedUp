package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ClawDebug extends CommandBase {

    CommandJoystick joy;
    Claw claw;
    double clawSpeed;


    public ClawDebug() {
        claw = RobotContainer.getClaw();
        joy = RobotContainer.getJoy();
        clawSpeed = .2;

        addRequirements(claw);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(joy.getHID().getRawButton(JoystickConstants.debugClawUp) 
        // && claw.getAngle() < clawConstants.UPPER_LIMIT_RAD
        ){
            claw.set(clawSpeed);
        } else if(joy.getHID().getRawButton(JoystickConstants.debugClawDown)
        // && claw.getAngle() > clawConstants.LOWER_LIMIT_RAD
        ){
            claw.set(-clawSpeed);
        } else{
            claw.stop();
        }

        if(joy.getHID().getRawButtonPressed(JoystickConstants.debugClawExtend)){
            claw.grab(!claw.getGrabber());
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted){
        claw.stop();
    }
}
