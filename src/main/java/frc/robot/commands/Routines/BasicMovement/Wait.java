package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double time;
    private Timer timer;
    public Wait(double time) { 
        this.time = time;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > time);
    }
}
