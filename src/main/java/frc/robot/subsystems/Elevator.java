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

    public CANSparkMax leftSpark;
    public CANSparkMax rightSpark;
    public Encoder throughbore;
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
        throughbore = new Encoder(0, 0);
        throughbore.setDistancePerPulse(ElevatorConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 8192);
        limitLeft = new DigitalInput(ElevatorConstants.LEFT_LIMIT_ID);
        limitRight = new DigitalInput(ElevatorConstants.RIGHT_LIMIT_ID);
        setPoint = 0;
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.FF_S, ElevatorConstants.FF_G, ElevatorConstants.FF_V, ElevatorConstants.FF_A); //FIXME
        elevatorController = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, 
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
        if (s < 0) {
            this.setPoint = 0;
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
