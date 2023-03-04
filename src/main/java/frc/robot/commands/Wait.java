package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double time;
    private Timer timer;
    private double heading;
    public Wait(double time, double heading) { 
        this.time = time;
        this.heading = heading;
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
