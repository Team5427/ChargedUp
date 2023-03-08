package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Claw;

public class ManualClaw extends CommandBase{
    private Claw claw;
    private double speed;

    public ManualClaw(double speed){
        claw = RobotContainer.getClaw();
        this.speed = speed;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        claw.set(speed);
    }

    @Override
    public boolean isFinished(){
        return (!RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CLAW_INTAKE) && 
        !RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CLAW_OUTTAKE));
    }

    @Override
    public void end(boolean interrupted){
        claw.stop();
    }
    
}
