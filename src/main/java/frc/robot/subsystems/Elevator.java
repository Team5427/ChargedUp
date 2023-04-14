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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.RobotContainer;
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
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        leftMotor.setInverted(true);
        leftMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS);
        setBrake(true);
        throughbore = new Encoder(ElevatorConstants.THROUGHBORE_ID_A, ElevatorConstants.THROUGHBORE_ID_B);
        throughbore.reset();
        throughbore.setDistancePerPulse(ElevatorConstants.POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 2048);
        limitLeft = new DigitalInput(ElevatorConstants.LEFT_LIMIT_ID);
        limitRight = new DigitalInput(ElevatorConstants.RIGHT_LIMIT_ID);
        setPoint = 0;
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
        leftMotor.set(value);
        rightMotor.set(value);
    }

    public boolean atGoal() {
        return (Math.abs(getHeight() - setPoint) < ElevatorConstants.GOAL_TOLERANCE_METERS);
    }

    public boolean atGoal(double pos) {
        return (Math.abs(getHeight() - pos) < ElevatorConstants.GOAL_TOLERANCE_METERS);
    }

    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean atLowerLimit(){
        return limitLeft.get() && limitRight.get();
    }

    public boolean atUpperLimit() {
        return (getHeight() >= ElevatorConstants.UPPER_LIMIT_METERS);
    }

    public void resetPIDs() {
        elevatorController.reset(getHeight());
    }

    @Override
    public void periodic() {
        if (!DriverStation.isEnabled() || RobotContainer.getOperatorJoy2().getHID().getRawButton(JoystickConstants.ELEVATOR_RESET)) {
            elevatorController.reset(getHeight());
            stop();
        } else {
            elevatorController.setGoal(this.setPoint);
            double calc = elevatorController.calculate(getHeight());
            set(calc);
        }

        if (atLowerLimit()) {
            resetEncoder();
        }


        // DEBUG CODE: DO NOT DELETE
        // if(RobotContainer.getOperatorJoy3().getHID().getRawButton(5)){
        //     set(.1);
        // } else if(RobotContainer.getOperatorJoy3().getHID().getRawButton(6)){
        //     set(-.1);
        // } else{
        //     set(0);
        // }

        log();
    }

    private void log() {
        Logger.post("Elevator Encoder", getHeight());
        Logger.post("left limit", limitLeft.get());
        Logger.post("Right limit", limitRight.get());
    }
    
}
