package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class Elevator extends SubsystemBase {

    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;
    public Encoder throughbore;
    public DigitalInput limitLeft;
    public DigitalInput limitRight;
    public double setPoint; //meters
    public ElevatorFeedforward elevatorFF;
    public ProfiledPIDController elevatorController;

    public Elevator() {
        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightMotor.setInverted(true);
        leftMotor.setInverted(true);
        setBrake(true); //FIXME turn to true after testing
        leftMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        throughbore = new Encoder(ElevatorConstants.THROUGHBORE_ID_A, ElevatorConstants.THROUGHBORE_ID_B);
        throughbore.reset();
        throughbore.setDistancePerPulse(ElevatorConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 2048);
        limitLeft = new DigitalInput(ElevatorConstants.LEFT_LIMIT_ID);
        limitRight = new DigitalInput(ElevatorConstants.RIGHT_LIMIT_ID);
        setPoint = 0;
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA); //FIXME
        elevatorController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
            new Constraints(ElevatorConstants.MAX_SPEED_M_S, ElevatorConstants.MAX_ACCEL_M_S_S)
        );
        elevatorController.setTolerance(ElevatorConstants.GOAL_TOLERANCE_METERS);
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
        double clamped = MathUtil.clamp(s, 0, ElevatorConstants.UPPER_LIMIT_METERS);
        this.setPoint = clamped;
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
        if (value <= 1) {
            if (!atLowerLimit() && !atUpperLimit()) {
                leftMotor.set(value);
                rightMotor.set(value);
            } else {
                if (atLowerLimit()) {
                    leftMotor.set(value <= 0 ? 0 : value);
                    rightMotor.set(value <= 0 ? 0 : value);
                } else {
                    leftMotor.set(value >= 0 ? 0 : value);
                    rightMotor.set(value >= 0 ? 0 : value);
                }
            }
        } else {
            stop();
        }
    }

    public boolean atGoal() {
        return (Math.abs(getHeight() - setPoint) < ElevatorConstants.GOAL_TOLERANCE_METERS);
    }

    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean atLowerLimit(){
        return limitLeft.get() || limitRight.get();
    }

    public boolean atUpperLimit() {
        return (getHeight() >= ElevatorConstants.UPPER_LIMIT_METERS);
    }

    @Override
    public void periodic() {
        double calc = elevatorController.calculate(getHeight());
        set(calc);

        if (DriverStation.isEnabled()) {
            if (getHeight() < ElevatorConstants.UPPER_LIMIT_METERS) {
                elevatorController.setGoal(this.setPoint);
            } else {
                elevatorController.setGoal(getHeight() - 0.01);
            }
        }

        if (atLowerLimit()) {
            resetEncoder();
        }

        // if (RobotContainer.getJoy().getHID().getRawButton(8)) {
        //     setPoint = .6540754;
        // } else {
        //     setPoint = 0.0;
        // }

        log();
    }

    private void log() {
        Logger.post("position", getHeight());
        Logger.post("left limit", limitLeft.get());
        Logger.post("right limit", limitRight.get());
        Logger.post("error", elevatorController.getPositionError());
        Logger.post("at Goal", elevatorController.atGoal());
        Logger.post("output", elevatorController.calculate(getHeight()));
        Logger.post("elevator setpoint position", elevatorController.getSetpoint().position);
        Logger.post("elevator setpoint velocity", elevatorController.getSetpoint().velocity);
        Logger.post("FF output", elevatorFF.calculate(elevatorController.getSetpoint().velocity));
    }
    
}
