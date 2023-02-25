package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class RampPusher extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder throughbore;
    private boolean deployed;
    private ProfiledPIDController controller;

    public RampPusher() {
        leftMotor = new CANSparkMax(RampPusherConstants.LEFT_ID, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(RampPusherConstants.CURRENT_LIMIT_AMPS);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor = new CANSparkMax(RampPusherConstants.RIGHT_ID, MotorType.kBrushless);
        rightMotor.setSmartCurrentLimit(RampPusherConstants.CURRENT_LIMIT_AMPS);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setInverted(true);
        throughbore = new DutyCycleEncoder(RampPusherConstants.THROUGHBORE_ID);
        throughbore.reset();
        throughbore.setPositionOffset(0);
        throughbore.setConnectedFrequencyThreshold(MiscConstants.THORUGHBORE_CONNECTION_HZ);
        controller = new ProfiledPIDController(RampPusherConstants.P, RampPusherConstants.I, RampPusherConstants.D, 
            new Constraints(RampPusherConstants.MAX_SPEED_RAD_S, RampPusherConstants.MAX_ACCEL_RAD_S_S)
        );
        controller.enableContinuousInput(0 - RampPusherConstants.ENCODER_OFFSET_RAD, (2 * Math.PI) - RampPusherConstants.ENCODER_OFFSET_RAD);
        controller.setTolerance(RampPusherConstants.CONTROLLER_TOLERANCE_RAD);
        deployed = true;
    }

    public void deploy(boolean bool) {
        deployed = bool;
    }

    public void move(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
    
    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean atSetpoint() {
        return ((deployed && (getPosition() < 0.2)) || (!deployed && (getPosition() > 1.4)));
    }

    public boolean isDeployed() {        
        return deployed;
    }


    public double getPosition() {
        return ((throughbore.getAbsolutePosition() * Math.PI * 2.0) - RampPusherConstants.ENCODER_OFFSET_RAD);
    }

    @Override
    public void periodic() {
        if (throughbore.isConnected()) {
            double calc = controller.calculate(getPosition());
            move(calc);
            if (DriverStation.isEnabled()) {
                if(deployed){
                    controller.setGoal(RampPusherConstants.DEPLOYED_POS_RAD);
                } else {
                    controller.setGoal(RampPusherConstants.UNDEPLOYED_POS_RAD);
                }
            } else {
                controller.setGoal(getPosition());
            }
        } else {
            stop();
        }
        log();
    }

    private void log() {
        Logger.post("ramp pusher angle", getPosition());
        // Logger.post("absolute position", throughbore.getAbsolutePosition());
        // Logger.post("controller setpoint", controller.getSetpoint().position);
        // Logger.post("THroughbore connect", throughbore.isConnected());
        // Logger.post("getFreq", throughbore.getFrequency());
    }
}
