package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;

public class UseIntake extends CommandBase{
    private Intake intake;

    private boolean intaking;
    public static boolean isRunning;

    private Timer timer;

    public UseIntake(){
        intake = RobotContainer.getIntake();
        timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize(){
        isRunning = true;
        if(intake.getProxCovered()){
            RobotContainer.getLed().setState(Led.INTAKE_FLOOR);
            intaking = false;
            timer.reset();
            timer.start();
        } else{
            RobotContainer.getLed().setState(Led.INTAKE_FLOOR);
            intake.setDeployed(true);
            intaking = true;
        }
    }

    @Override
    public void execute(){
        if(intaking){
            intake.intake();
        } else if(!intaking){
            intake.outtake();
        }
    }

    @Override 
    public boolean isFinished(){

        if(RobotContainer.getLed().getState() == Led.INTAKE || RobotContainer.getLed().getState() == Led.SCORING){
            return true;
        }

        if (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.SS_CANCEL)|| RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D)) {
            return true;
        }

        if (intake.getRetracted()) {
            return true;
        } else {
            if(intaking){
                return intake.getProxCovered();
            } else{
                return timer.get() >= IntakeConstants.OUTTAKE_TIME && !intake.getProxCovered();
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        isRunning = false;
        if(RobotContainer.getLed().getState() == Led.INTAKE || RobotContainer.getLed().getState() == Led.SCORING){
            intake.setDeployed(false);
        }else if (!intaking || !intake.getProxCovered()) {
            RobotContainer.getLed().setState(Led.INTAKE_FLOOR);
            intake.setDeployed(false);
        } else {
            RobotContainer.getLed().setState(Led.SCORING_FLOOR);
        }
        intake.stopIntake();
        intake.stopTilt();
        timer.stop();
        timer.reset();
    }
}