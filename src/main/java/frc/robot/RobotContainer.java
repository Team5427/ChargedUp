// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.RampPusherDebug;
import frc.robot.commands.Auton.AutonSheet;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RampPusher;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class RobotContainer {

  private static SwerveDrive swerveDrive;
  private static WPI_Pigeon2 pigeon;
  private static CommandJoystick joy;
  private static Limelight limelight;
  private static OdometryMath2023 odom;
  private static RampPusher rampPusher;

  public RobotContainer() {
    new Logger();

    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    joy = new CommandJoystick(Constants.JoystickConstants.DRIVER_CONTROLLER);
    limelight = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-steelta"));

    swerveDrive = new SwerveDrive(pigeon);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    rampPusher = new RampPusher();
    rampPusher.setDefaultCommand(new RampPusherDebug());

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
  public static Limelight getLimelight() {return limelight;}
  public static OdometryMath2023 getOdomInstance() {return odom;}
  public static RampPusher getRampPusher(){return rampPusher;}
}
