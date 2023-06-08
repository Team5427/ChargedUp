package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.GAME_PIECE_STATE;
import frc.robot.RobotContainer;
import frc.robot.commands.UseClaw;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class Claw extends SubsystemBase {

    CANSparkMax left;
    CANSparkMax right;
    AnalogInput sensor;
    Solenoid grabber;

    public Claw() {
        left = new CANSparkMax(ClawConstants.LEFT_ID, MotorType.kBrushless);
        left.setSmartCurrentLimit(ClawConstants.CURRENT_LIMIT_AMPS);
        left.setIdleMode(IdleMode.kBrake);
        right = new CANSparkMax(ClawConstants.RIGHT_ID, MotorType.kBrushless);
        right.setSmartCurrentLimit(ClawConstants.CURRENT_LIMIT_AMPS);
        right.setIdleMode(IdleMode.kBrake);
        OdometryMath2023.doPeriodicFrame(left, right);
        right.setInverted(true);
        left.setInverted(false);
        sensor = new AnalogInput(ClawConstants.PROX_ID);
        grabber = new Solenoid(28, PneumaticsModuleType.CTREPCM, ClawConstants.SOL_ID);
    }

    public ClawConstants.GAME_PIECE_STATE getState(boolean isPurple) {
        if (proxCovered()) { //has game piece
            if (isPurple) {
                return ClawConstants.GAME_PIECE_STATE.CUBE;
            } else {
                return GAME_PIECE_STATE.CONE;
            }
        } else {
            return ClawConstants.GAME_PIECE_STATE.NO_GP;
        }
    }

    public boolean proxCovered(){
        return getProx() > ClawConstants.PROX_VALUE;
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    public void grab(boolean closed) {
        grabber.set(!closed);
    }

    public boolean getGrabber() {
        return !grabber.get();
    }

    public void toggleGrabber(){
        grabber.toggle();
    }

    public double getProx(){
        return sensor.getAverageVoltage();
    }

    @Override
    public void periodic() {
        // Logger.post("debug blue abc", sensor.getBlue());
        Logger.post("claw debug prox voltage", getProx());

        Logger.post("State", getState(RobotContainer.getLed().isPurple()).toString());
        // Logger.post("claw sensor average voltage", sensor.getAverageVoltage());
        if (!UseClaw.isRunning) {
            if (RobotContainer.getController().getRightTriggerAxis() >= 0.15) {
                set(-RobotContainer.getController().getRightTriggerAxis());
            } else if (RobotContainer.getController().getLeftTriggerAxis() >= 0.15) {
                set(RobotContainer.getController().getLeftTriggerAxis());
            } else {
                set(0.015);
            }
        }

        if(getProx() == 0.0){
            RobotContainer.getLed().setError(true);
        } else {
            RobotContainer.getLed().setError(false);
        }

        if (MoveBotTo.goodToRelease && MoveClawTo.goodToRelease) {
            CommandScheduler.getInstance().schedule(new UseClaw());
            MoveBotTo.goodToRelease = false;
            MoveClawTo.goodToRelease = false;
        }
    }
}
