package frc.robot.subsystems;

import java.util.Set;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;

public class TankDrive {
    private CANSparkMax frontLeft, frontRight, backLeft, backRight;
    private MotorControllerGroup leftGroup, rightGroup;
    private Set<CANSparkMax> motors;
    private DifferentialDrive tankDrive;

    public TankDrive(AHRS gyro) {
        frontLeft = new CANSparkMax(Constants.TankConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.TankConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        backLeft = new CANSparkMax(Constants.TankConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);
        backRight = new CANSparkMax(Constants.TankConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);
        motors = Set.of(frontLeft, frontRight, backLeft, backRight);
        leftGroup = new MotorControllerGroup(frontLeft, backLeft);
        
    }
}
