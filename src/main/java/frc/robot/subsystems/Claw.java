package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.UseClaw;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class Claw extends SubsystemBase {

    CANSparkMax left;
    CANSparkMax right;
    ColorSensorV3 sensor;
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
        sensor = new ColorSensorV3(Port.kMXP);
        grabber = new Solenoid(28, PneumaticsModuleType.REVPH, ClawConstants.SOL_ID);
    }

    public ClawConstants.GAME_PIECE_STATE getState(boolean isPurple) {
        if (sensor.getProximity() > ClawConstants.PROX_VALUE) { //has game piece
            if (isPurple) {
                return ClawConstants.GAME_PIECE_STATE.CUBE;
            } else {
                return ClawConstants.GAME_PIECE_STATE.CONE;
            }
        } else {
            return ClawConstants.GAME_PIECE_STATE.NO_GP;
        }
    }

    public boolean proxCovered(){
        return sensor.getProximity() > ClawConstants.PROX_VALUE;
    }

    public boolean isPurple() {
        return (this.sensor.getBlue() > ClawConstants.PURPLE_THRESH);
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
        grabber.set(closed);
    }

    public boolean getGrabber() {
        return grabber.get();
    }

    @Override
    public void periodic() {
        Logger.post("claw state", getState(RobotContainer.getLed().isPurple()).name());
        Logger.post("claw clamped", getGrabber());
        // Logger.post("debug blue abc", sensor.getBlue());
        Logger.post("claw debug prox", sensor.getProximity());
        if (UseClaw.isRunning == false) {
            set(0.025);
        }

        // Logger.post("sensor connected", sensor.isConnected());
    }
}
