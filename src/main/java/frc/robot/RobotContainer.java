// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArmDebug;
import frc.robot.commands.ClawDebug;
import frc.robot.commands.ElevatorDebug;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.RampPusherDebug;
import frc.robot.commands.Auton.AutonSheet;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RampPusher;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class RobotContainer {

  private static SwerveDrive swerveDrive;
  private static RampPusher rampPusher;
  private static Elevator elevator;
  private static Arm arm;
  private static Claw claw;
  private static PneumaticHub hub;

  private static WPI_Pigeon2 pigeon;
  private static CommandJoystick joy;
  private static Limelight limelight_right, limelight_left;
  private static OdometryMath2023 odom;


  public RobotContainer() {
    new Logger();

    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    joy = new CommandJoystick(Constants.JoystickConstants.DRIVER_CONTROLLER);
    limelight_right = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-right"));
    limelight_left = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-left"));

    swerveDrive = new SwerveDrive(pigeon);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    rampPusher = new RampPusher();
    elevator = new Elevator();
    elevator.setDefaultCommand(new ElevatorDebug());
    arm = new Arm();
    arm.setDefaultCommand(new ArmDebug());
    claw = new Claw();
    claw.setDefaultCommand(new ClawDebug());

    hub = new PneumaticHub(28);
    hub.enableCompressorDigital();

    odom = new OdometryMath2023();

    //NEED TO BE AT END OF CONSTRUCTOR
    SwervePathMaker.initPaths("ComplexPath", "StraightLinePath");
    AutonSheet.initAutons();
    new ButtonBindings(getJoy());
  }

  public Command getAutonomousCommand() {
    return AutonSheet.testAuton;
  }

  public static SwerveDrive getSwerve() {return swerveDrive;}
  public static WPI_Pigeon2 getPigeon() {return pigeon;}
  public static CommandJoystick getJoy() {return joy;}
  public static Limelight getLimelightRight() {return limelight_right;}
  public static Limelight getLimelightLeft() {return limelight_left;}
  public static OdometryMath2023 getOdomInstance() {return odom;}
  public static RampPusher getRampPusher(){return rampPusher;}
  public static Elevator getElevator(){return elevator;}
  public static Arm getArm(){return arm;}
  public static Claw getClaw() {return claw;}
}
