package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {

    public CANSparkMax leftSpark;
    public CANSparkMax rightSpark;
    public RelativeEncoder throughbore;
    public DigitalInput limitLeft;
    public DigitalInput limitRight;
    public double setPoint; //meters
    public ElevatorFeedforward elevatorFF;
    public ProfiledPIDController elevatorController;

    public Elevator() {
        leftSpark = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightSpark = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightSpark.setInverted(true);
        setBrake(true);
        leftSpark.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        rightSpark.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        throughbore = leftSpark.getEncoder(Type.kQuadrature, 8192);
        limitLeft = new DigitalInput(ElevatorConstants.LEFT_LIMIT_ID);
        limitRight = new DigitalInput(ElevatorConstants.RIGHT_LIMIT_ID);
        throughbore.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS);
        throughbore.setVelocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS);
        setPoint = ElevatorConstants.LOWER_LIMIT_METERS;
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.FF_S, ElevatorConstants.FF_G, ElevatorConstants.FF_V, ElevatorConstants.FF_A); //FIXME
        elevatorController = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, 
            new Constraints(ElevatorConstants.MAX_SPEED_M_S, ElevatorConstants.MAX_ACCEL_M_S_S)
        );
        elevatorController.setTolerance(ElevatorConstants.GOAL_TOLERANCE_METERS);
    }

    public void resetEncoder() {
        throughbore.setPosition(ElevatorConstants.LOWER_LIMIT_METERS);
    }

    public double getHeight() {
        return throughbore.getPosition();
    }

    public double getSpeed() {
        return throughbore.getVelocity();
    }

    public void setHeight(double s) {
        if (s < ElevatorConstants.LOWER_LIMIT_METERS) {
            this.setPoint = ElevatorConstants.LOWER_LIMIT_METERS;
        } else {
            this.setPoint = s;
        }
    }

    public void setBrake(boolean braked) {
        IdleMode idle = braked ? IdleMode.kBrake : IdleMode.kCoast;
        leftSpark.setIdleMode(idle);
        rightSpark.setIdleMode(idle);
    }

    public boolean getBraked() {
        return (leftSpark.getIdleMode().equals(IdleMode.kBrake) && rightSpark.getIdleMode().equals(IdleMode.kBrake));
    }

    public void set(double value) {
        leftSpark.set(value);
        rightSpark.set(value);
    }

    public boolean atGoal() {
        return elevatorController.atGoal();
    }

    // @Override
    // public void periodic() {

    //     if (getHeight() <= ElevatorConstants.UPPER_LIMIT_METERS) {
    //         set(elevatorController.calculate(getHeight(), setPoint) + elevatorFF.calculate(elevatorController.getSetpoint().velocity));
    //     } else {
    //         set(0);
    //     }

    //     if (limitLeft.get() || limitRight.get()) {
    //         resetEncoder();
    //     }
    // }
    
}
