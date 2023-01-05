package frc.robot.subsystems;

import java.util.Set;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDrive extends SubsystemBase {

    private CANSparkMax frontLeft, frontRight, backLeft, backRight;
    private RelativeEncoder leftEnc, rightEnc;
    private PIDController leftPID, rightPID;
    private SimpleMotorFeedforward tankFF;
    private MotorControllerGroup leftGroup, rightGroup;
    private Set<CANSparkMax> motors;
    private DifferentialDrive tankDrive;
    private DifferentialDriveOdometry odometry;
    private AHRS m_gyro;

    public TankDrive(AHRS gyro) {
        this.m_gyro = gyro;
        frontLeft = new CANSparkMax(Constants.TankConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.TankConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        motors.forEach((m) -> {
            m.setIdleMode(IdleMode.kBrake);
        });
        backLeft = new CANSparkMax(Constants.TankConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);
        backLeft.follow(frontLeft);
        backRight = new CANSparkMax(Constants.TankConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);
        backRight.follow(frontRight);
        leftEnc = frontLeft.getEncoder();
        leftEnc.setPositionConversionFactor(Constants.TankConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS);
        leftEnc.setVelocityConversionFactor(Constants.TankConstants.VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS);
        leftEnc.setPosition(0);
        rightEnc = frontRight.getEncoder();
        rightEnc.setPositionConversionFactor(Constants.TankConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS);
        rightEnc.setVelocityConversionFactor(Constants.TankConstants.VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS);
        leftGroup = new MotorControllerGroup(frontLeft, backLeft);
        rightGroup = new MotorControllerGroup(frontRight,backRight);
        tankDrive = new DifferentialDrive(leftGroup, rightGroup);
        leftPID = new PIDController(Constants.TankConstants.DRIVE_P, 0, 0);
        rightPID = new PIDController(Constants.TankConstants.DRIVE_P, 0, 0);
        tankFF = new SimpleMotorFeedforward(Constants.TankConstants.DRIVE_KS, Constants.TankConstants.DRIVE_KV, Constants.TankConstants.DRIVE_KA);
        odometry = new DifferentialDriveOdometry(getRotation2d(), leftPos(), rightPos());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {return new DifferentialDriveWheelSpeeds(leftSpeed(), rightSpeed());}
    public double leftSpeed(){return leftEnc.getVelocity();}
    public double rightSpeed(){return rightEnc.getVelocity();}
    public double leftPos(){return leftEnc.getPosition();}
    public double rightPos(){return rightEnc.getPosition();}
    public PIDController getLeftPID(){return leftPID;}
    public PIDController getRightPID(){return rightPID;}
    public SimpleMotorFeedforward getFF(){return tankFF;}
    public Rotation2d getRotation2d(){return m_gyro.getRotation2d();}

    public void joystickDrive(XboxController controller) { //add calculations here
        double speedX = controller.getLeftY();
        double speedRot = Math.copySign(Math.pow(controller.getRightX(), Constants.TankConstants.JOYSTICK_TURNING_EXPONENT), controller.getRightX());
        var speeds = DifferentialDrive.arcadeDriveIK(speedX, speedRot, false);
        setSpeeds(speeds.left * Constants.TankConstants.MAX_VELOCITY_MPS, speeds.right * Constants.TankConstants.MAX_VELOCITY_MPS);
    }

    public void setSpeeds(double leftMetersPerSecond, double rightMetersPerSecond) {
        leftGroup.setVoltage(tankFF.calculate(leftMetersPerSecond) + leftPID.calculate(leftMetersPerSecond, leftMetersPerSecond));
        rightGroup.setVoltage(tankFF.calculate(rightMetersPerSecond) + rightPID.calculate(rightMetersPerSecond, rightMetersPerSecond));
    }

    public void setVoltage(double l, double r) {
        leftGroup.setVoltage(l);
        rightGroup.setVoltage(r);
    }

    public void stopMotors() {
        leftGroup.stopMotor();
        rightGroup.stopMotor();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), leftPos(), rightPos(), pose);
    }

    public void setBrake(boolean brake) {
        motors.forEach((m) -> {
            if (brake) {
                m.setIdleMode(IdleMode.kBrake);
            } else {
                m.setIdleMode(IdleMode.kCoast);
            }
        });
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), leftPos(), rightPos());
        tankDrive.feed();
    }
}
