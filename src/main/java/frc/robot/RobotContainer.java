// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.Auton.AutonSheet;
import frc.robot.commands.Auton.SubRoutineSheet;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ExtraLight;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class RobotContainer {

  private static SwerveDrive swerveDrive;
  private static Elevator elevator;
  private static Arm arm;
  private static Claw claw;
  private static Led led;
  private static Intake intake;

  private static PneumaticsControlModule hub;

  private static WPI_Pigeon2 pigeon;
  private static CommandJoystick joy;
  private static CommandJoystick operatorJoy1;
  private static CommandJoystick operatorJoy2;
  private static CommandXboxController xboxController;
  // private static CommandJoystick operatorJoy3;
  private static Limelight limelight_right, limelight_left;
  private static ExtraLight tapelight;
  private static OdometryMath2023 odom;
  private static SendableChooser<SequentialCommandGroup> autonSelector;


  public RobotContainer() {
    new Logger();

    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    // joy = new CommandJoystick(Constants.JoystickConstants.DRIVER_CONTROLLER);
    operatorJoy1 = new CommandJoystick(Constants.JoystickConstants.OPERATION_CONTROLLER);
    operatorJoy2 = new CommandJoystick(JoystickConstants.OPERATION2_CONTROLLER);
    xboxController = new CommandXboxController(0);

    swerveDrive = new SwerveDrive(pigeon);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    elevator = new Elevator();
    arm = new Arm();
    claw = new Claw();
    intake = new Intake();

    hub = new PneumaticsControlModule(28);
    hub.enableCompressorDigital();
    limelight_right = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-right"), 
      List.of(Math.toRadians(1.2), Math.toRadians(1.2), Math.toRadians(1.2)), //offset on tags 6, 7, 8 respectively
      List.of(0.0, 0.0, 0.0) //3, 2, 1
    );
    limelight_left = new Limelight(NetworkTableInstance.getDefault().getTable("limelight-left"), 
      List.of(Math.toRadians(2.5), Math.toRadians(2.5), Math.toRadians(2.5)), //offset on tags 6, 7, 8 respectively
      List.of(0.0, 0.0, 0.0) //3, 2, 1
    );

    tapelight = new ExtraLight(NetworkTableInstance.getDefault().getTable("limelight-front"));

    odom = new OdometryMath2023();

    led = new Led();

    autonSelector = new SendableChooser<>();

    //NEED TO BE AT END OF CONSTRUCTOR
    SwervePathMaker.initPaths(
      "TopSingleConeIntakeEngage1", 
      "TopSingleConeIntake1",
      "TopDoubleConeScore1",
      "TopDoubleConeScore2",
      "BottomSingleConeIntakeEngage1",
      "TopDoubleConeEngage1",
      "TopDoubleConeEngage2",
      "FloorTripleCleanSide1",
      "FloorTripleCleanSide2",
      "FloorTripleBumpSide1",
      "FloorTripleBumpSide2",
      "IntegratedAuton1",
      "IntegratedAuton2"
    );
    AutonSheet.initAutons();
    SubRoutineSheet.initSubRoutines();


    // autonSelector.setDefaultOption("Engage",AutonSheet.topSingleConeIntakeEngage); 
    autonSelector.addOption("Score only",AutonSheet.topSingleConeIntake);
    autonSelector.setDefaultOption("2 piece pickup",AutonSheet.topDoubleConeScore);
    autonSelector.addOption("BOTTOM SIDE INTAKE", AutonSheet.bottomSingleConeIntakeEngage);
    autonSelector.addOption("2 piece engage", AutonSheet.topDoubleConeEngage);
    autonSelector.addOption("Middle Auton", AutonSheet.robonautsAuton);
    autonSelector.addOption("triple bump side", AutonSheet.floorTripleBumpSide);

    Logger.postComplex("Auton Paths", autonSelector);
    new ButtonBindings(getOperatorJoy1(), getOperatorJoy2(), getController());
  }

  public Command getAutonomousCommand() {
    return autonSelector.getSelected();
    // return null;
  }

  public static SwerveDrive getSwerve() {return swerveDrive;}
  public static WPI_Pigeon2 getPigeon() {return pigeon;}
  public static CommandJoystick getOperatorJoy1() {return operatorJoy1;}
  public static CommandJoystick getOperatorJoy2() {return operatorJoy2;}
  public static CommandXboxController getController() {return xboxController;}
  public static Limelight getLimelightRight() {return limelight_right;}
  public static Limelight getLimelightLeft() {return limelight_left;}
  public static ExtraLight getLimelightTape() {return tapelight;}
  public static OdometryMath2023 getOdomInstance() {return odom;}
  public static Elevator getElevator(){return elevator;}
  public static Arm getArm(){return arm;}
  public static Claw getClaw() {return claw;}
  public static Led getLed(){return led;}
  public static Intake getIntake(){return intake;}
}
