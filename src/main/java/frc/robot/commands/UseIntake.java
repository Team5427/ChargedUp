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

        if (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.SS_CANCEL)) {
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
        if (!intaking || !intake.getProxCovered()) {
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
