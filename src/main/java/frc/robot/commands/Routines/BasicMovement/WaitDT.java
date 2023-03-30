package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WaitDT extends CommandBase{

    private Timer clock;
    private double time;

    public WaitDT(double time) {
        addRequirements(RobotContainer.getSwerve());
        this.time = time;
        clock = new Timer();

    }

    @Override
    public void initialize() {
        clock.reset();
        clock.start();
    }

    @Override
    public boolean isFinished() {
        return clock.get() >= time;
    }

    @Override
    public void end(boolean interrupted) {
        clock.reset();
    }
    
}
