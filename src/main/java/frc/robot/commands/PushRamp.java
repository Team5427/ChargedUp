package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class PushRamp extends CommandBase {

    private Intake intake;
    public static boolean isRunning;

    public PushRamp() {
        intake = RobotContainer.getIntake();
        addRequirements(intake);
        isRunning = false;
    }

    @Override
    public void initialize() {
        isRunning = true;
        intake.setManualSetpoint(IntakeConstants.RAMP_PUSHING_SETPOINT_RAD);
        intake.stopIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setManualSetpoint(Double.NaN);
        isRunning = false;
    }
}
