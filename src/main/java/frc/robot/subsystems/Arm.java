package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class Arm extends SubsystemBase {
    
    private CANSparkMax topMotor;
    private CANSparkMax btmMotor;
    private DutyCycleEncoder throughbore;
    private double setPoint;
    private double calc;
    private ArmFeedforward armFF;
    private ProfiledPIDController armController;
    private Solenoid sol;

    public Arm() {
        topMotor = new CANSparkMax(ArmConstants.TOP_ID, MotorType.kBrushless);
        btmMotor = new CANSparkMax(ArmConstants.BTM_ID, MotorType.kBrushless);
        btmMotor.setInverted(false);
        topMotor.setInverted(true);
        topMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        btmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        setBrake(true);
        throughbore = new DutyCycleEncoder(ArmConstants.THROUGHBORE_ID);
        throughbore.reset();
        throughbore.setPositionOffset(0);
        armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
        armController = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
            new Constraints(ArmConstants.MAX_SPEED_RAD_S, ArmConstants.MAX_ACCEL_RAD_S_S)
        );
        armController.enableContinuousInput(0 - ArmConstants.POSITION_OFFSET_RAD, (2 * Math.PI) - ArmConstants.POSITION_OFFSET_RAD);
        armController.setTolerance(ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD);
        armController.reset(getAngle());
        setPoint = ArmConstants.UPPER_LIMIT_RAD; //arm locks up on robot startup
        sol = new Solenoid(28, PneumaticsModuleType.REVPH, ArmConstants.SOL_ID);
    }

    public double getAngle() {
        return ((throughbore.getAbsolutePosition() * Math.PI * 2) - ArmConstants.POSITION_OFFSET_RAD);
    }

    public void setAngle(double s) {
        this.setPoint = MathUtil.clamp(s, ArmConstants.LOWER_LIMIT_RAD, ArmConstants.UPPER_LIMIT_RAD);
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
        if (getAngle() == ArmConstants.POSITION_OFFSET_RAD) {
            stopMotors();
            System.out.println("DANGER ARM ENCODER NOT PLUGGED IN");
        } else {
            topMotor.set(speed);
            btmMotor.set(speed);    
        }
    }

    public void stopMotors() {
        topMotor.stopMotor();
        btmMotor.stopMotor();
    }

    public boolean atGoal() {
        return (Math.abs(getAngle() - setPoint) < ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD);
    }

    public boolean atJankGoal() {
        return (Math.abs(getAngle() - setPoint) < ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD_JANK);
    }

    public boolean atGoal(double pos) {
        return (Math.abs(getAngle() - pos) < ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD);
    }

    public boolean getExtended() {
        return sol.get();
    }

    public void extend(boolean extended) {
        sol.set(extended);
    }

    public void stop(){
        topMotor.stopMotor();
        btmMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (atJankGoal() && (setPoint == ArmConstants.UPPER_LIMIT_RAD)) {
            set(0.02);
            armController.reset(getAngle());
        } else {
            calc = armController.calculate(getAngle());
            set(calc);    
        }
        if (DriverStation.isEnabled()) { 
            armController.setGoal(this.setPoint);
        } else {
            armController.setGoal(getAngle());
        }
        log();
    }

    public void log() {
        // Logger.post("calculation", this.calc);
        Logger.post("arm angle", getAngle());
        // Logger.post("at goal", atGoal());
        // Logger.post("arm setpoint", this.setPoint);
        // Logger.post("bruh", true);
    }
}
