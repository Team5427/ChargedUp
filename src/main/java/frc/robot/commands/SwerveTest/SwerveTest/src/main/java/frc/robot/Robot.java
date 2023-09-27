// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//hello eric
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax turnMotor;
  private CANSparkMax speedMotor;
    //hello eric
  private CANCoder m_canCoder;

  private RelativeEncoder turnEncoder;
  private RelativeEncoder speedEncoder;

  private SparkMaxPIDController turnControl;
  private SparkMaxPIDController speedControl;

  private CommandXboxController joy;

  private double setPointSpeed;
  private double setPointAngleRad;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    turnMotor = new CANSparkMax(Constants.TURN_MOTOR_ID, MotorType.kBrushless);
    speedMotor = new CANSparkMax(Constants.SPEED_MOTOR_ID, MotorType.kBrushless);

    turnMotor.restoreFactoryDefaults();
    speedMotor.restoreFactoryDefaults();
    Timer.delay(0.2);

    turnEncoder = turnMotor.getEncoder();
    turnEncoder.setPositionConversionFactor(Constants.TURN_CONVERSION_FACTOR_ROT_TO_RAD);
    speedEncoder = speedMotor.getEncoder();
    speedEncoder.setVelocityConversionFactor(Constants.SPEED_CONVERSION_FACTOR_RPM_TO_MPS);

    m_canCoder = new CANCoder(Constants.CANCODER_ID);
    m_canCoder.configFactoryDefault(200);
    m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    turnControl = turnMotor.getPIDController();
    turnControl.setPositionPIDWrappingEnabled(true);
    turnControl.setPositionPIDWrappingMaxInput(2 * Math.PI);
    turnControl.setPositionPIDWrappingMinInput(0);

    speedControl = speedMotor.getPIDController();
    speedControl.setSmartMotionMaxAccel(Constants.MAX_CONTROL_ACCEL_MPSS, 0);

    turnControl.setP(Constants.TURN_KP);
    turnControl.setD(Constants.TURN_KD);

    speedControl.setP(Constants.DRIVE_KP, 0);
    speedControl.setFF(Constants.DRIVE_FF, 0);

    turnEncoder.setPosition(Units.degreesToRadians(m_canCoder.getAbsolutePosition()));
    speedEncoder.setPosition(0);

    joy = new CommandXboxController(0);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    setPointSpeed = Math.hypot(joy.getRightX(), joy.getRightY()) * Constants.MAX_CONTROL_SPEED_MPS;
    setPointAngleRad = Math.atan2(joy.getRightY(), joy.getRightX());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (joy.getHID().getRawButton(1)) {
      turnControl.setReference(setPointAngleRad, ControlType.kPosition);
      speedControl.setReference(setPointSpeed, ControlType.kSmartVelocity);
    } else {
      turnMotor.stopMotor();
      speedControl.setReference(0, ControlType.kSmartVelocity);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
    //hello eric
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}    //hello eric
