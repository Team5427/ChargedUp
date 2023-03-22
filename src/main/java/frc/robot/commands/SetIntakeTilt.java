package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class SetIntakeTilt extends CommandBase{
    private Intake intake;
    private double angle;

    public SetIntakeTilt(double angle){
        intake = RobotContainer.getIntake();
        this.angle = angle;

        addRequirements(intake);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){

        if(intake.getAngle() < angle + IntakeConstants.TOLERANCE_RAD){
            intake.setTilt(IntakeConstants.INTAKE_SPEED);
        } else if(intake.getAngle() > angle - IntakeConstants.TOLERANCE_RAD){
            intake.setTilt(-IntakeConstants.INTAKE_SPEED);
        }
    }

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.stopIntake();
        intake.stopTilt();
        
    }
}
