package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class RampPusher extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder throughbore;
    private boolean deployed;
    private ProfiledPIDController controller;

    public RampPusher() {

        // LEFT MOTOR IS LEADER

        leftMotor = new CANSparkMax(RampPusherConstants.LEFT_ID, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(50);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor = new CANSparkMax(RampPusherConstants.RIGHT_ID, MotorType.kBrushless);
        rightMotor.setSmartCurrentLimit(50);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setInverted(true);

        rightMotor.follow(leftMotor);

        throughbore = new DutyCycleEncoder(0);
        throughbore.setDistancePerRotation(Math.PI * 2);
        throughbore.setPositionOffset((throughbore.getAbsolutePosition() * Math.PI * 2) - RampPusherConstants.ENCODER_OFFSET_RAD);
        controller = new ProfiledPIDController(RampPusherConstants.P, RampPusherConstants.I, RampPusherConstants.D, 
            new Constraints(RampPusherConstants.MAX_SPEED_RAD_S, RampPusherConstants.MAX_ACCEL_RAD_S_S)
        );
        deployed = false;
    }

    public void deploy(boolean bool) {
        deployed = bool;

        if(deployed){
            leftMotor.set(controller.calculate(getPosition(), Constants.RampPusherConstants.DEPLOYED_POS_RAD));
        } else{
            leftMotor.set(controller.calculate(getPosition(), Constants.RampPusherConstants.UNDEPLOYED_POS_RAD));
        }
    }

    public void move(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
    
    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isDeployed() {
        return deployed;
    }


    public double getPosition() {
        return throughbore.getDistance();
    }

    @Override
    public void periodic() {
        // if (deployed) {
        //     controller.calculate(getPosition(), RampPusherConstants.DEPLOYED_POS_RAD);
        // } else {
        //     controller.calculate(getPosition(), RampPusherConstants.UNDEPLOYED_POS_RAD);
        // }
        log();
    }

    private void log() {
        Logger.post("ramp pusher positon", getPosition());
    }


}
