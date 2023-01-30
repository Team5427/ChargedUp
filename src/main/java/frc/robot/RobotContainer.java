// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.Auton.AutonSheet;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2022;
import frc.robot.util.SwervePathMaker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static SwerveDrive swerveDrive;
  private static Pigeon2 pigeon;
  private static Joystick joy;
  private static Limelight limelight;
  private static OdometryMath2022 odom;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    pigeon = new Pigeon2(Constants.PIGEON_ID);
    pigeon.configFactoryDefault();
    joy = new Joystick(Constants.DRIVER_CONTROLLER);
    limelight = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-scrappy"));

    swerveDrive = new SwerveDrive(pigeon);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    odom = new OdometryMath2022();

    //NEED TO BE AT END OF CONSTRUCTOR
    SwervePathMaker.initPaths("swervePath1", "swervePath2");
    AutonSheet.initAutons();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutonSheet.testAuton;
  }

  public static SwerveDrive getSwerve() {return swerveDrive;}
  public static Pigeon2 getPigeon() {return pigeon;}
  public static Joystick getJoy() {return joy;}
  public static Limelight getLimelight() {return limelight;}
  public static OdometryMath2022 getOdomInstance() {return odom;}
}
