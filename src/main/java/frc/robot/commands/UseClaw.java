package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Led;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class UseClaw extends CommandBase {

    private Claw claw;
    private Led led;
    private ClawConstants.GAME_PIECE_STATE initState;
    private boolean finish = false;
    private boolean intake;
    private boolean isPurple;
    private Timer timer;
    public static boolean isRunning;

    public UseClaw() {
        claw = RobotContainer.getClaw();
        led = RobotContainer.getLed();
        addRequirements(claw);
        timer = new Timer();
    }

    public UseClaw(boolean intake) {
        claw = RobotContainer.getClaw();
        led = RobotContainer.getLed();
        addRequirements(claw);
        timer = new Timer();
        this.intake = intake;
    }

    @Override
    public void initialize() {
        isRunning = true;
        timer.reset();
        finish = false;
        isPurple = led.isPurple();
        // isPurple = claw.isPurple();
        initState = claw.getState(isPurple);
        if ((initState.equals(ClawConstants.GAME_PIECE_STATE.NO_GP)) || (initState.equals(ClawConstants.GAME_PIECE_STATE.CONE) && !claw.getGrabber())) {
            intake = true;
        } else {
            intake = false;
        }
        Logger.post("Claw finished", false);

    }

    @Override
    public void execute() {
        
        isPurple = led.isPurple();
        // isPurple = claw.isPurple();
        if (intake) {
            claw.grab(false);
            claw.set(ClawConstants.INTAKE_SPEED_DECIMAL);
            if (claw.getState(isPurple).equals(ClawConstants.GAME_PIECE_STATE.CONE)) {
                claw.grab(true);
                finish = true;
            } else if (claw.getState(isPurple).equals(ClawConstants.GAME_PIECE_STATE.CUBE)) {
                claw.grab(false);
                timer.start();
                if (timer.get() > ClawConstants.CUBE_INTAKE_EXCESS_TIME_S) {
                    finish = true;
                }
            }
        } else if (initState.equals(ClawConstants.GAME_PIECE_STATE.CONE)) {
            claw.grab(false);
            finish = true;
        } else {
            if (OdometryMath2023.inScoringSpot()) {
                claw.set(ClawConstants.OUTTAKE_SPEED_DECIMAL);
            } else {
                claw.set(ClawConstants.OUTTAKE_SPEED_DECIMAL_SHOOTING);
            }
            timer.start();
            if (timer.get() > ClawConstants.CUBE_OUTTAKE_EXCESS_TIME_S) {
                finish = true;
            }
        }
        

    }

    @Override
    public boolean isFinished() {
        return (finish || RobotContainer.getJoy().getHID().getRawButtonPressed(JoystickConstants.SS_CANCEL));
    }

    @Override
    public void end(boolean interrupted) {
        if(intake){
            led.setState(Led.SCORING);
        } else{
            led.setState(Led.INTAKE);
        }

        if (!DriverStation.isAutonomous()) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new WaitCommand(.3),
                new MoveClawTo(RoutineConstants.DEFAULT_CLAW_STATE)

                ));
        }

        isRunning = false;
        timer.stop();
        timer.reset();
        claw.stop();
    }
}
