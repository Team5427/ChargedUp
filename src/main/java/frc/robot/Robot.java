// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.pathUtil.SwervePathMaker;
import frc.robot.subsystems.Led;
import frc.robot.util.OdometryMath2023;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer matchTimer;
  public static boolean specialAuton;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    matchTimer = new Timer();
    m_robotContainer = new RobotContainer();
    RobotContainer.getSwerve().setHeadingRad(Math.PI);
    specialAuton = false;

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SwervePathMaker.resetPaths();
    RobotContainer.getSwerve().setBrake(true, true);
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.getArm().resetPIDs();
    RobotContainer.getElevator().resetPIDs();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.getLed().setState(Led.INTAKE);
    RobotContainer.getSwerve().stopMods();
    RobotContainer.getSwerve().resetMods();
    RobotContainer.getSwerve().setBrake(true, true);
    
    RobotContainer.getLed().setPurple(false);
    RobotContainer.getClaw().grab(true);
    RobotContainer.getElevator().resetPIDs();
    RobotContainer.getArm().resetPIDs();

    Pose2d bluePose = new Pose2d(0, 0, new Rotation2d(Math.PI));
    if (OdometryMath2023.isBlue()) {
      RobotContainer.getSwerve().resetOdometry(bluePose);
    } else {
      RobotContainer.getSwerve().resetOdometry(OdometryMath2023.flip(bluePose));
    }

    matchTimer.reset();
    matchTimer.start();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (matchTimer.get() > 14.5) {
      RobotContainer.getIntake().outtake(-1.0);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.getSwerve().setFieldRelative(true);
    RobotContainer.getSwerve().resetMods();
    RobotContainer.getLed().setState(Led.INTAKE);
    RobotContainer.getClaw().grab(true);
    RobotContainer.getSwerve().setBrake(true, true);
    RobotContainer.getIntake().stopIntake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


}
