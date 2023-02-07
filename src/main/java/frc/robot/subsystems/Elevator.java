package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {

    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;
    public Encoder throughbore;
    public DigitalInput limitLeft;
    public DigitalInput limitRight;
    public double setPoint; //meters
    public ElevatorFeedforward elevatorFF;
    public ProfiledPIDController leftElevatorController, rightElevatorController;

    public Elevator() {
        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightMotor.setInverted(true);
        setBrake(true);
        leftMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        throughbore = new Encoder(0, 0);
        throughbore.reset();
        throughbore.setDistancePerPulse(ElevatorConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 8192);
        limitLeft = new DigitalInput(ElevatorConstants.LEFT_LIMIT_ID);
        limitRight = new DigitalInput(ElevatorConstants.RIGHT_LIMIT_ID);
        setPoint = 0;
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA); //FIXME
        leftElevatorController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
            new Constraints(ElevatorConstants.MAX_SPEED_M_S, ElevatorConstants.MAX_ACCEL_M_S_S)
        );
        leftElevatorController.setTolerance(ElevatorConstants.GOAL_TOLERANCE_METERS);
        rightElevatorController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
            new Constraints(ElevatorConstants.MAX_SPEED_M_S, ElevatorConstants.MAX_ACCEL_M_S_S)
        );
        rightElevatorController.setTolerance(ElevatorConstants.GOAL_TOLERANCE_METERS);
    }

    public void resetEncoder() {
        throughbore.reset();
    }

    public double getHeight() {
        return throughbore.getDistance();
    }

    public double getSpeed() {
        return throughbore.getRate();
    }

    public void setHeight(double s) {
        if (s < 0) {
            this.setPoint = 0;
        } else {
            this.setPoint = s;
        }
    }

    public void setBrake(boolean braked) {
        IdleMode idle = braked ? IdleMode.kBrake : IdleMode.kCoast;
        leftMotor.setIdleMode(idle);
        rightMotor.setIdleMode(idle);
    }

    public boolean getBraked() {
        return (leftMotor.getIdleMode().equals(IdleMode.kBrake) && rightMotor.getIdleMode().equals(IdleMode.kBrake));
    }

    public void setV(double value) {
        leftMotor.setVoltage(value);
        rightMotor.setVoltage(value);
    }

    public void set(double value) {
        leftMotor.set(value);
        rightMotor.set(value);
    }

    public boolean atGoal() {
        return leftElevatorController.atGoal();
    }

    public double getPosition() {
        return throughbore.getDistance();
    }

    @Override
    public void periodic() {

        if (getHeight() <= ElevatorConstants.UPPER_LIMIT_METERS) {
            leftElevatorController.setGoal(setPoint);
            rightElevatorController.setGoal(setPoint);
        } else {
            leftElevatorController.setGoal(getPosition());
            rightElevatorController.setGoal(getPosition());
        }

        if (limitLeft.get() || limitRight.get()) {
            resetEncoder();
        }
    }
    
}
