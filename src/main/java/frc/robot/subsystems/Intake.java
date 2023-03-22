package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.Logger;

public class Intake extends SubsystemBase{

    public CANSparkMax intakeMotor;
    public AnalogInput prox;
    
    public CANSparkMax tiltMotor;
    private DutyCycleEncoder throughbore;

    public boolean deployed;

    public Intake(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kCoast);

        prox = new AnalogInput(IntakeConstants.PROX_ID);

        tiltMotor = new CANSparkMax(IntakeConstants.TILT_ID, MotorType.kBrushless);
        tiltMotor.setInverted(false);
        tiltMotor.setIdleMode(IdleMode.kCoast);

        throughbore = new DutyCycleEncoder(IntakeConstants.THROUGHBORE_ID);
        throughbore.reset();
        throughbore.setPositionOffset(0);

        deployed = false;
    }
    
    public double getAngle() {
        return ((throughbore.getAbsolutePosition() * Math.PI * 2) - IntakeConstants.ENCODER_OFFSET_RAD);
    }

    public void setTilt(double speed){
        tiltMotor.set(speed);
    }

    public void stopTilt(){
        tiltMotor.stopMotor();
    }

    public void setDeployed(boolean deployed){
        this.deployed = deployed;
    }

    public boolean getDeployed(){
        return deployed;
    }

    public void setIntake(double speed){
        intakeMotor.set(speed);
    }

    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    public double getProximity(){
        return prox.getAverageVoltage();
    }

    public boolean getProxCovered(){
        return getProximity() < IntakeConstants.INTAKE_COVERED;
    }

    @Override
    public void periodic(){
        log();
    }
    
    private void log(){
        Logger.post("deployed", deployed);
        Logger.post("intake angle", getAngle());
    }
    
}