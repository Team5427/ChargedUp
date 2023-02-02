package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    
    private CANSparkMax topMotor;
    private CANSparkMax btmMotor;
    private DutyCycleEncoder throughbore;
    private double setPoint;
    private ArmFeedforward armFF;
    private ProfiledPIDController armController;

    public Arm() {
        topMotor = new CANSparkMax(0, MotorType.kBrushless);
        btmMotor = new CANSparkMax(0, MotorType.kBrushless);
        btmMotor.setInverted(true);
        topMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        btmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        setBrake(true);
        throughbore = new DutyCycleEncoder(ArmConstants.THROUGHBORE_ID);
        throughbore.setPositionOffset(ArmConstants.POSITION_OFFSET_RAD);
        throughbore.setDistancePerRotation(Math.PI * 2);
        armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
        armController = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
            new Constraints(ArmConstants.MAX_SPEED_RAD_S, ArmConstants.MAX_ACCEL_RAD_S_S)
        );
        armController.setTolerance(ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD);
        setPoint = getAngle(); //arm locks up on robot startup
    }

    public double getAngle() {
        return throughbore.getDistance();
    }

    public void setAngle(double s) {
        this.setPoint = s;
    }

    public void setBrake(boolean braked) {
        IdleMode idle = braked ? IdleMode.kBrake : IdleMode.kCoast;
        topMotor.setIdleMode(idle);
        btmMotor.setIdleMode(idle);
    }

    public void setV(double speed) {
        topMotor.setVoltage(speed);
        btmMotor.setVoltage(speed);
    }

    public void set(double speed) {
        topMotor.set(speed);
        btmMotor.set(speed);
    }

    public int inRange() {
        boolean ret = (getAngle() < ArmConstants.UPPER_LIMIT_RAD);
        boolean ret2 = (getAngle() > ArmConstants.LOWER_LIMIT_RAD);
        if (ret && ret2) {
            return 1;
        } else if (ret) {
            return 2; //too low
        } else if (ret2) {
            return 3; //too high
        } else {
            return 4;
        }
    }

    public boolean atGoal() {
        return armController.atGoal();
    }

    // @Override
    // public void periodic() {
    //     if (inRange() == 1) {
    //         setV(armController.calculate(getAngle(), setPoint) + armFF.calculate(getAngle(), armController.getSetpoint().velocity));
    //     } else if (inRange() == 2) {
    //         setV(0.1); //might have to negate
    //     } else if (inRange() == 3) {
    //         setV(-.1);
    //     } else {
    //         setV(0);
    //         System.out.println("theres some zesty shit going on with this arm bro i swr.");
    //     }
    // }
}
