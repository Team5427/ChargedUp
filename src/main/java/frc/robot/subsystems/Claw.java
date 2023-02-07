package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.util.OdometryMath2023;

public class Claw extends SubsystemBase {

    CANSparkMax left;
    CANSparkMax right;
    ColorSensorV3 sensor;

    public Claw() {
        left = new CANSparkMax(0, MotorType.kBrushless);
        left.setSmartCurrentLimit(ClawConstants.CURRENT_LIMIT_AMPS);
        OdometryMath2023.doPeriodicFrame(left);
        right = new CANSparkMax(0, MotorType.kBrushless);
        right.setSmartCurrentLimit(ClawConstants.CURRENT_LIMIT_AMPS);
        OdometryMath2023.doPeriodicFrame(right);
        right.setInverted(true);
        sensor = new ColorSensorV3(Port.kOnboard);
    }

    public ClawConstants.GAME_PIECE_STATE getState() {
        if (sensor.getProximity() < ClawConstants.PROX_VALUE) { //has game piece
            if (isPurple(sensor)) {
                return ClawConstants.GAME_PIECE_STATE.CUBE;
            } else if (!isPurple(sensor)) {
                return ClawConstants.GAME_PIECE_STATE.CONE;
            } else {
                return ClawConstants.GAME_PIECE_STATE.NO_GP;
            }
        } else {
            return ClawConstants.GAME_PIECE_STATE.NO_GP;
        }
    }

    public boolean isPurple(ColorSensorV3 sensor) {
        return (sensor.getBlue() > ClawConstants.PURPLE_THRESH);
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }


}
