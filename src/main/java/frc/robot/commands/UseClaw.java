package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class UseClaw extends CommandBase {

    private Claw claw;
    private ClawConstants.GAME_PIECE_STATE initState;
    private boolean finish = false;
    private boolean intake;
    private Timer timer;

    public UseClaw() {
        // claw = RobotContainer.getClaw()
        addRequirements(claw);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        initState = claw.getState();
        if (initState.equals(ClawConstants.GAME_PIECE_STATE.NO_GP)) {
            intake = true;
        } else {
            intake = false;
        }
    }

    @Override
    public void execute() {
        if (intake) {
            claw.set(ClawConstants.INTAKE_SPEED_DECIMAL);
            if (claw.getState().equals(ClawConstants.GAME_PIECE_STATE.CONE)) {
                claw.grab(true);
                finish = true;
            } else if (claw.getState().equals(ClawConstants.GAME_PIECE_STATE.CUBE)) {
                timer.start();
                if (timer.get() > ClawConstants.CUBE_INTAKE_EXCESS_TIME_S) {
                    finish = true;
                }
            }
        } else if (initState.equals(ClawConstants.GAME_PIECE_STATE.CONE)) {
            claw.grab(false);
            finish = true;
        } else {
            claw.set(ClawConstants.OUTTAKE_SPEED_DECIMAL);
            timer.start();
            if (timer.get() > ClawConstants.CUBE_OUTTAKE_EXCESS_TIME_S) {
                finish = true;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return (finish);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        claw.stop();
    }
}
