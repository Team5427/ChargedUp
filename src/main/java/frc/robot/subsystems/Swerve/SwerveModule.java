package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.*;

public class SwerveModule {

    private int speedMotorID;
    private int turnMotorID;
    private int absEncID;
    private boolean speedInv;
    private boolean turnInv;
    private boolean encInv;
    private CANSparkMax speedMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder speedEnc;
    private RelativeEncoder turnEnc;
    private CANCoder absEnc;
    private ProfiledPIDController turningPID;
    private SimpleMotorFeedforward turningFF;
    private PIDController speedPID;
    private SimpleMotorFeedforward speedFF;
    private double encoderOffset;

    public SwerveModule (SwerveConstants.SwerveModuleType type) {
        determineIDs(type);
        init();
    }

    public CANSparkMax getDriveSpark() {return speedMotor;}
    public CANSparkMax getTurnSpark() {return turnMotor;}
    public PIDController getDrivePID() {return speedPID;}
    public SimpleMotorFeedforward getDriveFF() {return speedFF;}
    public ProfiledPIDController getTurnPID() {return turningPID;}
    public SimpleMotorFeedforward getTurnFF() {return turningFF;}
    public CANCoder getAbsEnc() {return absEnc;}
    public double getDrivePos() {return speedEnc.getPosition();}
    public double getDriveSpeed() {return speedEnc.getVelocity();}
    public double getTurnPosRad() {return turnEnc.getPosition();}
    public double getTurnVel() {return turnEnc.getVelocity();}
    public double getAbsEncRaw() {return Math.toRadians(absEnc.getAbsolutePosition());}
    public double getAbsEncRad() {
        double x = getAbsEncRaw();
        x -= encoderOffset;
        x = encInv ? (x * -1): x;
        return x;
    }


    public SwerveModuleState getModState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getAbsEncRad()));
    }

    public SwerveModulePosition getModPosition() {
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getAbsEncRad()));
    }

    public double backToRPM() {
        return getDriveSpeed()/SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S;
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) <= 0.02) {
            stop();
        } else {
            SwerveModuleState newState;
            
            newState = SwerveModuleState.optimize(state, getModState().angle);
            // speedMotor.setVoltage(speedPID.calculate(getDriveSpeed(), state.speedMetersPerSecond) + speedFF.calculate(state.speedMetersPerSecond));
            speedMotor.set(newState.speedMetersPerSecond / SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC);
            turnMotor.setVoltage(turningPID.calculate(getAbsEncRad(), newState.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
        }
    }

    public void hardSetModState(SwerveModuleState state){
        SwerveModuleState newState;
        newState = SwerveModuleState.optimize(state, getModState().angle);
            // speedMotor.setVoltage(speedPID.calculate(getDriveSpeed(), state.speedMetersPerSecond) + speedFF.calculate(state.speedMetersPerSecond));
            speedMotor.set(newState.speedMetersPerSecond / SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC);
            turnMotor.setVoltage(turningPID.calculate(getAbsEncRad(), newState.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
    }

    public void stop() {
        speedMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void setBrake(boolean driveBrake, boolean steerBrake) {
        speedMotor.setIdleMode(driveBrake ? IdleMode.kBrake : IdleMode.kCoast);
        turnMotor.setIdleMode(steerBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    private void determineIDs(SwerveConstants.SwerveModuleType type) {
        switch(type) {
            case FRONT_LEFT:
                speedMotorID = SwerveConstants.FRONT_LEFT_DRIVE_MOTOR;
                turnMotorID = SwerveConstants.FRONT_LEFT_STEER_MOTOR;
                absEncID = SwerveConstants.FRONT_LEFT_CANCODER;
                encoderOffset = SwerveConstants.FRONT_LEFT_OFFSET;
                speedInv = SwerveConstants.FRONT_LEFT_DRIVE_INVERT;
                turnInv = SwerveConstants.FRONT_LEFT_TURNING_INVERT;
                encInv = SwerveConstants.FRONT_LEFT_CANCODER_INVERT;
                break;
            case FRONT_RIGHT:
                speedMotorID = SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR;
                turnMotorID = SwerveConstants.FRONT_RIGHT_STEER_MOTOR;
                absEncID = SwerveConstants.FRONT_RIGHT_CANCODER;
                encoderOffset = SwerveConstants.FRONT_RIGHT_OFFSET;
                speedInv = SwerveConstants.FRONT_RIGHT_DRIVE_INVERT;
                turnInv = SwerveConstants.FRONT_RIGHT_TURNING_INVERT;
                encInv = SwerveConstants.FRONT_RIGHT_CANCODER_INVERT;
                break;
            case BACK_LEFT:
                speedMotorID = SwerveConstants.BACK_LEFT_DRIVE_MOTOR;
                turnMotorID = SwerveConstants.BACK_LEFT_STEER_MOTOR;
                absEncID = SwerveConstants.BACK_LEFT_CANCODER;
                encoderOffset = SwerveConstants.BACK_LEFT_OFFSET;
                speedInv = SwerveConstants.BACK_LEFT_DRIVE_INVERT;
                turnInv = SwerveConstants.BACK_LEFT_TURNING_INVERT;
                encInv = SwerveConstants.BACK_LEFT_CANCODER_INVERT;
                break;
            case BACK_RIGHT:
                speedMotorID = SwerveConstants.BACK_RIGHT_DRIVE_MOTOR;
                turnMotorID = SwerveConstants.BACK_RIGHT_STEER_MOTOR;
                absEncID = SwerveConstants.BACK_RIGHT_CANCODER;
                encoderOffset = SwerveConstants.BACK_RIGHT_OFFSET;
                speedInv = SwerveConstants.BACK_RIGHT_DRIVE_INVERT;
                turnInv = SwerveConstants.BACK_RIGHT_TURNING_INVERT;
                encInv = SwerveConstants.BACK_RIGHT_CANCODER_INVERT;
                break;
        }
    }
    
    public void init() {
        speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        speedMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        speedMotor.setSmartCurrentLimit(37);
        turnMotor.setSmartCurrentLimit(20);
        speedMotor.setInverted(speedInv);
        turnMotor.setInverted(turnInv);
        speedEnc = speedMotor.getEncoder();
        turnEnc = turnMotor.getEncoder();
        absEnc = new CANCoder(absEncID);
        setBrake(false, false);
        turningPID = new ProfiledPIDController(SwerveConstants.TURNING_PID_P, SwerveConstants.TURNING_PID_D, 0, 
            new Constraints(SwerveConstants.TURNING_MAX_SPEED_RAD_S, SwerveConstants.TURNING_MAX_ACCEL_RAD_S_S));
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        turningFF = new SimpleMotorFeedforward(SwerveConstants.TURNING_FF_S, SwerveConstants.TURNING_FF_V, SwerveConstants.TURNING_FF_A);
        speedPID = new PIDController(SwerveConstants.SPEED_PID_P, 0, 0);
        speedFF = new SimpleMotorFeedforward(SwerveConstants.SPEED_FF_S, SwerveConstants.SPEED_FF_V, SwerveConstants.SPEED_FF_A);
        speedEnc.setPositionConversionFactor(SwerveConstants.SWERVE_CONVERSION_FACTOR_ROT_TO_METER);
        speedEnc.setVelocityConversionFactor(SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(SwerveConstants.SWERVE_CONVERSION_FACTOR_ROT_TO_RAD);
        turnEnc.setVelocityConversionFactor(SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        speedEnc.setPosition(0);
        absEnc.configFactoryDefault();
        absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEnc.setPosition(getAbsEncRad());
    }
}
