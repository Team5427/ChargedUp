// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.Auton.AutonSheet;
import frc.robot.commands.Auton.SubRoutineSheet;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class RobotContainer {

  private static SwerveDrive swerveDrive;
  private static Elevator elevator;
  private static Arm arm;
  private static Claw claw;
  private static Led led;

  private static PneumaticHub hub;

  private static WPI_Pigeon2 pigeon;
  private static CommandJoystick joy;
  private static CommandJoystick operatorJoy1;
  private static CommandJoystick operatorJoy2;
  private static Limelight limelight_right, limelight_left;
  private static OdometryMath2023 odom;
  private static SendableChooser<SequentialCommandGroup> autonSelector;


  public RobotContainer() {
    new Logger();

    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    joy = new CommandJoystick(Constants.JoystickConstants.DRIVER_CONTROLLER);
    operatorJoy1 = new CommandJoystick(Constants.JoystickConstants.OPERATION_CONTROLLER);
    operatorJoy2 = new CommandJoystick(JoystickConstants.OPERATION2_CONTROLLER);

    swerveDrive = new SwerveDrive(pigeon);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    elevator = new Elevator();
    arm = new Arm();
    claw = new Claw();

    hub = new PneumaticHub(28);
    hub.enableCompressorDigital();
    limelight_right = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-right"));
    limelight_left = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-left"));

    odom = new OdometryMath2023();

    led = new Led();

    autonSelector = new SendableChooser<>();

    //NEED TO BE AT END OF CONSTRUCTOR
    SwervePathMaker.initPaths(
      "BottomSingleConeEngage1", 
      "TopSingleConeEngage1", 
      "TopSingleConeIntakeEngage1", 
      "BottomSingleCone1", 
      "TopDoubleConeEngage1", 
      "TopDoubleConeEngage2",
      "TopSingleConeIntake1",
      "BottomSingleConeIntakeEngage1"
    );
    AutonSheet.initAutons();
    SubRoutineSheet.initSubRoutines();

    autonSelector.addOption("BottomSingleConeEngage",AutonSheet.bottomSingleConeEngage); 
    autonSelector.addOption("TopSingleConeEngage",AutonSheet.topSingleConeEngage); 
    autonSelector.setDefaultOption("TopSingleConeIntakeEngage",AutonSheet.topSingleConeIntakeEngage); 
    autonSelector.addOption("BottomSingleCone",AutonSheet.bottomSingleCone); 
    autonSelector.addOption("TopDoubleConeEngage",AutonSheet.topDoubleConeEngage); 
    autonSelector.addOption("TopSingleConeIntake",AutonSheet.topSingleConeIntake);
    autonSelector.addOption("BottomSingleConeIntakeEngage",AutonSheet.bottomSingleConeIntakeEngage);    

    Logger.postComplex("Auton Paths", autonSelector);
    new ButtonBindings(getJoy(), getOperatorJoy1(), getOperatorJoy2());
  }

  public Command getAutonomousCommand() {
    return autonSelector.getSelected();
  }

  public static SwerveDrive getSwerve() {return swerveDrive;}
  public static WPI_Pigeon2 getPigeon() {return pigeon;}
  public static CommandJoystick getJoy() {return joy;}
  public static CommandJoystick getOperatorJoy1() {return operatorJoy1;}
  public static CommandJoystick getOperatorJoy2() {return operatorJoy2;}
  // public static CommandJoystick getOperatorJoy3() {return operatorJoy3;}
  public static Limelight getLimelightRight() {return limelight_right;}
  public static Limelight getLimelightLeft() {return limelight_left;}
  public static OdometryMath2023 getOdomInstance() {return odom;}
  public static Elevator getElevator(){return elevator;}
  public static Arm getArm(){return arm;}
  public static Claw getClaw() {return claw;}
  public static Led getLed(){return led;}
}
