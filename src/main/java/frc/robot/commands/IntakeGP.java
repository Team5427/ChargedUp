package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;

public class IntakeGP extends CommandBase{
    public Intake intake;
    public Claw claw;
    public Led led;

    public IntakeGP(){
        intake = RobotContainer.getIntake();
        claw = RobotContainer.getClaw();
        led = RobotContainer.getLed();
        addRequirements(intake, claw);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        if(led.getState() == led.INTAKE_FLOOR){
            CommandScheduler.getInstance().schedule(new UseIntake());
        } else{
            CommandScheduler.getInstance().schedule(new UseClaw());
        }
    }
}
