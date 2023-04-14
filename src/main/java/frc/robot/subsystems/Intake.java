package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.UseIntake;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class Intake extends SubsystemBase{

    private CANSparkMax intakeMotor;
    private AnalogInput prox;
    private SlewRateLimiter limiter;
    
    private CANSparkMax tiltMotorLeft;
    private CANSparkMax tiltMotorRight;
    private DutyCycleEncoder throughbore;

    private Rotation2d setPointRad;

    public boolean retracted;
    public boolean deployed;
    private double error;

    public Intake(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(35);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        OdometryMath2023.doPeriodicFrame(intakeMotor);
        
        limiter = new SlewRateLimiter(IntakeConstants.ACCEL_PERCENT);
        limiter.reset(0);

        prox = new AnalogInput(IntakeConstants.PROX_ID);

        tiltMotorLeft = new CANSparkMax(IntakeConstants.TILT_ID_LEFT, MotorType.kBrushless);
        tiltMotorLeft.restoreFactoryDefaults();
        tiltMotorLeft.setSmartCurrentLimit(25);
        tiltMotorLeft.setInverted(true);
        tiltMotorLeft.setIdleMode(IdleMode.kBrake);

        tiltMotorRight = new CANSparkMax(IntakeConstants.TILT_ID_RIGHT, MotorType.kBrushless);
        tiltMotorRight.restoreFactoryDefaults();
        tiltMotorRight.setSmartCurrentLimit(25);
        tiltMotorRight.setInverted(false);
        tiltMotorRight.setIdleMode(IdleMode.kBrake);

        retracted = false;
        deployed = false;

        throughbore = new DutyCycleEncoder(IntakeConstants.THROUGHBORE_ID);
        throughbore.reset();
        throughbore.setPositionOffset(0);
    }
    
    public double getAngle() {
        return ((throughbore.getAbsolutePosition() * Math.PI * 2) - IntakeConstants.ENCODER_OFFSET_RAD);
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getAngle());
    }

    public void setTilt(double speed){
        double calc = limiter.calculate(speed);
        tiltMotorLeft.set(calc);
        tiltMotorRight.set(calc);
    }

    public void stopTilt(){
        tiltMotorLeft.stopMotor();
        tiltMotorRight.stopMotor();
    }

    public void intake() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void outtake() {
        if (OdometryMath2023.inScoringSpot()) {
            intakeMotor.set(IntakeConstants.OUTTAKE_SPEED_SLOW);
        } else {
            intakeMotor.set(IntakeConstants.OUTTAKE_SPEED);
        }
    }

    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    public boolean getProxCovered(){
        return prox.getAverageVoltage() > IntakeConstants.INTAKE_COVERED;
    }

    public void setRetracted(boolean r) {
        this.retracted = r;
    }

    public void setDeployed(boolean d) {
        this.deployed = d;
    }

    public boolean getRetracted() {
        return retracted;
    }

    public boolean getDeployed() {
        return deployed;
    }

    public boolean atSetpoint() {
        return Math.abs(error) < IntakeConstants.TOLERANCE_RAD;
    }

    @Override
    public void periodic(){
        // if (retracted) {
        //     setPointRad = new Rotation2d(IntakeConstants.RETRACTED_POS_RAD);
        // } else {
        //     if (deployed) {
        //         setPointRad = new Rotation2d(IntakeConstants.DEPLOYED_POS_RAD);
        //     } else {
        //         setPointRad = new Rotation2d(IntakeConstants.UNDEPLOYED_POS_RAD);
        //     }
        // }
        // error = setPointRad.minus(getRotation2d()).getRadians();


        // if (Math.abs(RobotContainer.getArm().getAngle()) < IntakeConstants.ARM_CLEARANCE_RAD || RobotContainer.getArm().atJankGoal()) {
        //     if (Math.abs(error) > IntakeConstants.TOLERANCE_RAD) {
        //         setTilt(IntakeConstants.TILT_COEF * Math.signum(error));
        //     } else {
        //         stopTilt();
        //     }
        // } else {
        //     stopTilt();
        // }

        // if (!UseIntake.isRunning) {
        //     intakeMotor.set(IntakeConstants.STATIC_HOLD_SPEED);
        // }

        if(RobotContainer.getOperatorJoy3().getHID().getRawButton(3)){
            setTilt(.1);
        } else if(RobotContainer.getOperatorJoy3().getHID().getRawButton(4)){
            setTilt(-.1);
        } else{
            setTilt(0);
        }

        if(RobotContainer.getOperatorJoy3().getHID().getRawButton(1)){
            intake();
        } else if(RobotContainer.getOperatorJoy3().getHID().getRawButton(8)){
            outtake();
        } else{
            stopIntake();
        }

        log();
    }
    
    private void log(){
        Logger.post("intake angle", getAngle());

        Logger.post("intake Prox", prox.getAverageVoltage());
        
    }
    
}