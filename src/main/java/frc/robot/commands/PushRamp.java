package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class PushRamp extends CommandBase {

    private Intake intake;
    public static boolean isRunning;
    private boolean auton;
    private Timer timer;
    private double time;

    public PushRamp(boolean auton, double time) {
        intake = RobotContainer.getIntake();
        addRequirements(intake);
        isRunning = false;
        this.auton = auton;
        timer = new Timer();
        this.time = time;
    }

    @Override
    public void initialize() {
        isRunning = true;
        intake.setManualSetpoint(IntakeConstants.RAMP_PUSHING_SETPOINT_RAD);
        intake.stopIntake();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > time && auton);
    }


    @Override
    public void end(boolean interrupted) {
        intake.setManualSetpoint(Double.NaN);
        isRunning = false;
    }
}
