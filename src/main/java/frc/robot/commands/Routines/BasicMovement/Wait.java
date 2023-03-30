package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase{

    private Timer clock;
    private double time;

    public Wait(double time) {
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
