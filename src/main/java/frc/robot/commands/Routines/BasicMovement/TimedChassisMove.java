package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TimedChassisMove extends CommandBase {

    private SwerveDrive dt;
    private Translation2d setPoint;
    private double time;
    private Timer timer;

    public TimedChassisMove(Translation2d setPoint, double time) {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        timer = new Timer();
        this.time = time;
        this.setPoint = setPoint;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        dt.setChassisSpeeds(new ChassisSpeeds(setPoint.getX(), setPoint.getY(), 0));
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopMods();
    }
}
