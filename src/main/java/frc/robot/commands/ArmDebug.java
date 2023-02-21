package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmDebug extends CommandBase {

    CommandJoystick joy;
    Arm arm;
    double armSpeed;


    public ArmDebug() {
        arm = RobotContainer.getArm();
        joy = RobotContainer.getJoy();
        armSpeed = .2;

        addRequirements(arm);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(joy.getHID().getRawButton(JoystickConstants.debugArmUp) 
        // && arm.getAngle() < ArmConstants.UPPER_LIMIT_RAD
        ){
            arm.set(armSpeed);
        } else if(joy.getHID().getRawButton(JoystickConstants.debugArmDown)
        // && arm.getAngle() > ArmConstants.LOWER_LIMIT_RAD
        ){
            arm.set(-armSpeed);
        } else{
            arm.stop();
        }

        if(joy.getHID().getRawButtonPressed(JoystickConstants.debugArmExtend)){
            arm.extend(!arm.getExtended());
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted){
        arm.stop();
    }
}
