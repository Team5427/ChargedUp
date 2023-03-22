package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class UseIntake extends CommandBase{
    private Intake intake;

    private boolean intaking;

    private Timer timer;

    public UseIntake(){
        intake = RobotContainer.getIntake();
        timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize(){
        if(intake.getProxCovered()){
            intaking = false;
            timer.reset();
            timer.start();
        } else{
            intaking = true;
        }
    }

    @Override
    public void execute(){

        if(intake.getAngle() > IntakeConstants.DEPLOYED_POS_RAD){ //FIXME
            intake.setTilt(IntakeConstants.TILT_SPEED);
        } else{
            intake.stopTilt();
        }

        if(intaking && !intake.getProxCovered()){
            intake.setIntake(IntakeConstants.INTAKE_SPEED);
        } else if(!intaking && timer.get() < IntakeConstants.OUTTAKE_TIME){
            intake.setIntake(IntakeConstants.OUTTAKE_SPEED);
        }
    }

    @Override 
    public boolean isFinished(){
        if(intaking){
            return intake.getProxCovered();
        } else{
            return timer.get() >= IntakeConstants.OUTTAKE_TIME;
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.stopIntake();
        intake.stopTilt();
        timer.stop();
        timer.reset();
    }
}
