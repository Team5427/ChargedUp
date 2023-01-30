package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
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
import frc.robot.Constants;

public class SwerveModule {

    private int speedMotorID;
    private int turnMotorID;
    private int fckUp;
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

    public SwerveModule (Constants.SwerveConstants.SwerveModuleType type) {
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
        return getDriveSpeed()/Constants.SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S;
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) <= 0.02) {
            stop();
        } else {
            state = SwerveModuleState.optimize(state, getModState().angle);
            speedMotor.setVoltage(speedPID.calculate(getDriveSpeed(), state.speedMetersPerSecond) + speedFF.calculate(state.speedMetersPerSecond));
            turnMotor.setVoltage(turningPID.calculate(getAbsEncRad(), state.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
        }
    }

    public void stop() {
        speedMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void setBrake(boolean driveBrake, boolean steerBrake) {
        speedMotor.setIdleMode(driveBrake ? IdleMode.kBrake : IdleMode.kCoast);
        turnMotor.setIdleMode(steerBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public int getErrors() {
        return fckUp;
    }

    public void incrementError() {
        fckUp++;
    }

    private void determineIDs(Constants.SwerveConstants.SwerveModuleType type) {
        switch(type) {
            case FRONT_LEFT:
                speedMotorID = Constants.SwerveConstants.FRONT_LEFT_SPEED_MOTOR;
                turnMotorID = Constants.SwerveConstants.FRONT_LEFT_TURN_MOTOR;
                absEncID = Constants.SwerveConstants.FRONT_LEFT_CANCODER;
                encoderOffset = Constants.SwerveConstants.FRONT_LEFT_OFFSET;
                speedInv = Constants.SwerveConstants.FRONT_LEFT_DRIVE_INVERT;
                turnInv = Constants.SwerveConstants.FRONT_LEFT_TURNING_INVERT;
                encInv = Constants.SwerveConstants.FRONT_LEFT_CANCODER_INVERT;
                break;
            case FRONT_RIGHT:
                speedMotorID = Constants.SwerveConstants.FRONT_RIGHT_SPEED_MOTOR;
                turnMotorID = Constants.SwerveConstants.FRONT_RIGHT_TURN_MOTOR;
                absEncID = Constants.SwerveConstants.FRONT_RIGHT_CANCODER;
                encoderOffset = Constants.SwerveConstants.FRONT_RIGHT_OFFSET;
                speedInv = Constants.SwerveConstants.FRONT_RIGHT_DRIVE_INVERT;
                turnInv = Constants.SwerveConstants.FRONT_RIGHT_TURNING_INVERT;
                encInv = Constants.SwerveConstants.FRONT_RIGHT_CANCODER_INVERT;
                break;
            case BACK_LEFT:
                speedMotorID = Constants.SwerveConstants.BACK_LEFT_SPEED_MOTOR;
                turnMotorID = Constants.SwerveConstants.BACK_LEFT_TURN_MOTOR;
                absEncID = Constants.SwerveConstants.BACK_LEFT_CANCODER;
                encoderOffset = Constants.SwerveConstants.BACK_LEFT_OFFSET;
                speedInv = Constants.SwerveConstants.BACK_LEFT_DRIVE_INVERT;
                turnInv = Constants.SwerveConstants.BACK_LEFT_TURNING_INVERT;
                encInv = Constants.SwerveConstants.BACK_LEFT_CANCODER_INVERT;
                break;
            case BACK_RIGHT:
                speedMotorID = Constants.SwerveConstants.BACK_RIGHT_SPEED_MOTOR;
                turnMotorID = Constants.SwerveConstants.BACK_RIGHT_TURN_MOTOR;
                absEncID = Constants.SwerveConstants.BACK_RIGHT_CANCODER;
                encoderOffset = Constants.SwerveConstants.BACK_RIGHT_OFFSET;
                speedInv = Constants.SwerveConstants.BACK_RIGHT_DRIVE_INVERT;
                turnInv = Constants.SwerveConstants.BACK_RIGHT_TURNING_INVERT;
                encInv = Constants.SwerveConstants.BACK_RIGHT_CANCODER_INVERT;
                break;
        }
    }
    
    public void init() {
        speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        speedMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        speedMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(27);
        speedMotor.setInverted(speedInv);
        turnMotor.setInverted(turnInv);
        speedEnc = speedMotor.getEncoder();
        turnEnc = turnMotor.getEncoder();
        absEnc = new CANCoder(absEncID);
        setBrake(false, false);
        turningPID = new ProfiledPIDController(Constants.SwerveConstants.TURNING_PID_P, Constants.SwerveConstants.TURNING_PID_D, 0, 
            new Constraints(Constants.SwerveConstants.TURNING_MAX_SPEED_RAD_S, Constants.SwerveConstants.TURNING_MAX_ACCEL_RAD_S_S));
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        turningFF = new SimpleMotorFeedforward(Constants.SwerveConstants.TURNING_FF_S, Constants.SwerveConstants.TURNING_FF_V, Constants.SwerveConstants.TURNING_FF_A);
        speedPID = new PIDController(Constants.SwerveConstants.SPEED_PID_P, 0, 0);
        speedFF = new SimpleMotorFeedforward(Constants.SwerveConstants.SPEED_FF_S, Constants.SwerveConstants.SPEED_FF_V, Constants.SwerveConstants.SPEED_FF_A);
        speedEnc.setPositionConversionFactor(Constants.SwerveConstants.SWERVE_CONVERSION_FACTOR_ROT_TO_METER);
        speedEnc.setVelocityConversionFactor(Constants.SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(Constants.SwerveConstants.SWERVE_CONVERSION_FACTOR_ROT_TO_RAD);
        turnEnc.setVelocityConversionFactor(Constants.SwerveConstants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        speedEnc.setPosition(0);
        absEnc.configFactoryDefault();
        absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEnc.setPosition(getAbsEncRad());
        fckUp = 0;
    }
}
