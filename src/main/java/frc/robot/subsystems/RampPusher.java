package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RampPusher extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder throughbore;
    private boolean deployed;
    private ProfiledPIDController controller;

    public RampPusher() {
        leftMotor = new CANSparkMax(Constants.RampPusherConstants.LEFT_ID, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(50);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor = new CANSparkMax(Constants.RampPusherConstants.RIGHT_ID, MotorType.kBrushless);
        rightMotor.setSmartCurrentLimit(50);
        rightMotor.setIdleMode(IdleMode.kBrake);
        throughbore = new DutyCycleEncoder(0);
        throughbore.setDistancePerRotation(Math.PI * 2);
        controller = new ProfiledPIDController(0, 0, 0, 
            new Constraints(0, 0)
        );
        deployed = false;
    }

    public void deploy(boolean bool) {
        deployed = bool;
    }

    public void move(double speed){
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }
    
    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isDeployed() {
        return deployed;
    }

    @Override
    public void periodic() {
        
        // if (deployed) {
        //     controller.calculate(throughbore.getDistance(), 0);
        // } else {
        //     controller.calculate(throughbore.getDistance(), 0);
        // }

        if(RobotContainer.getJoy().getHID().getRawButton(10)){
            move(Constants.RampPusherConstants.UP_SPEED);
        }
        else if(RobotContainer.getJoy().getHID().getRawButton(9)){
            move(Constants.RampPusherConstants.DOWN_SPEED);
        }
        else{
            stop();
        }
    }
}
