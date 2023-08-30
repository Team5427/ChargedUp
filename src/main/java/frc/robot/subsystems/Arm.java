package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class Arm extends SubsystemBase {
    
    private CANSparkMax topMotor;
    private CANSparkMax btmMotor;
    private DutyCycleEncoder throughbore;
    private double setPoint;
    private double calc;
    private ProfiledPIDController armController;
    private Solenoid sol;

    public Arm() {
        topMotor = new CANSparkMax(ArmConstants.TOP_ID, MotorType.kBrushless);
        btmMotor = new CANSparkMax(ArmConstants.BTM_ID, MotorType.kBrushless);
        topMotor.restoreFactoryDefaults();
        Timer.delay(0.02);
        btmMotor.restoreFactoryDefaults();
        Timer.delay(0.02);
        btmMotor.setInverted(false);
        topMotor.setInverted(true);
        topMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        btmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMPS);
        setBrake(true);
        throughbore = new DutyCycleEncoder(ArmConstants.THROUGHBORE_ID);
        throughbore.reset();
        throughbore.setPositionOffset(0);
        armController = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
            new Constraints(ArmConstants.MAX_SPEED_RAD_S, ArmConstants.MAX_ACCEL_RAD_S_S)
        );
        armController.enableContinuousInput(0 - ArmConstants.POSITION_OFFSET_RAD, (2 * Math.PI) - ArmConstants.POSITION_OFFSET_RAD);
        armController.setTolerance(ArmConstants.ARM_CONTROLLER_TOLERANCE_RAD);
        armController.reset(getAngle());
        setPoint = ArmConstants.UPPER_LIMIT_RAD; //arm locks up on robot startup
        armController.setGoal(this.setPoint);
        sol = new Solenoid(28, PneumaticsModuleType.CTREPCM, ArmConstants.SOL_ID);

        topMotor.burnFlash();
        Timer.delay(0.02);
        btmMotor.burnFlash();
        Timer.delay(0.02);

        OdometryMath2023.doPeriodicFrame(20, topMotor, btmMotor);
    }

    public double getAngle() {
        return ((throughbore.getAbsolutePosition() * Math.PI * 2) - ArmConstants.POSITION_OFFSET_RAD);
    }

    public void setAngle(double s) {
        this.setPoint = MathUtil.clamp(s, ArmConstants.LOWER_LIMIT_RAD, ArmConstants.UPPER_LIMIT_RAD);
        armController.setGoal(this.setPoint);
    }

    public void setBrake(boolean braked) {
        IdleMode idle = braked ? IdleMode.kBrake : IdleMode.kCoast;
        topMotor.setIdleMode(idle);
        btmMotor.setIdleMode(idle);
    }

    public void set(double speed) {
        if (getAngle() == ArmConstants.POSITION_OFFSET_RAD) {
            stopMotors();
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
        if (getExtended()) {
            armController.setConstraints(new Constraints(ArmConstants.MAX_SPEED_RAD_S, ArmConstants.MAX_ACCEL_RAD_S_S_EXTENDED));
        } else {
            armController.setConstraints(new Constraints(ArmConstants.MAX_SPEED_RAD_S, ArmConstants.MAX_ACCEL_RAD_S_S));
        }
    }

    public void stop(){
        topMotor.stopMotor();
        btmMotor.stopMotor();
    }

    public void resetPIDs() {
        armController.reset(getAngle());
    }

    @Override
    public void periodic() {

        if (getAngle() == -ArmConstants.POSITION_OFFSET_RAD) {
            set(0.025);
        } else {
            if (atJankGoal() && (setPoint == ArmConstants.UPPER_LIMIT_RAD)) {
                set(0.025);
                resetPIDs();
            } else {
                calc = armController.calculate(getAngle());
                set(calc);    
            }
        }
        
        log();
    }

    public void log() {
        // Logger.post("calculation", this.calc);
        Logger.post("arm angle", getAngle());
        // Logger.post(" goal", atJankGoal());
        Logger.post("arm encoder plugged in", throughbore.isConnected());
        // Logger.post("arm encoder frequency", throughbore.getFrequency());
        // Logger.post("bruh", true);
    }
}
