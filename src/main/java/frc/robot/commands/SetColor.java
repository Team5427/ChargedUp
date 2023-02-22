package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Led;

public class SetColor extends CommandBase{
    Led led = RobotContainer.getLed();
    public SetColor(){
        addRequirements(led);
    }

    @Override
    public void initialize(){
        led.togglePurple();
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
