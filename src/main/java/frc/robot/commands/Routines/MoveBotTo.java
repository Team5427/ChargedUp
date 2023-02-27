package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;

public class MoveBotTo extends CommandBase {

    private Pose2d setpoint;
    private Pose2d measurement;
    private SwerveDrive swerve;
    public static boolean isRunning;
    public static double runningSpeed = 0;
    private static ProfiledPIDController xController, yController, thetaController;
    private Timer timer;
    private static RoutineConstants.POSITION_TYPE lastPositionType;
    private RoutineConstants.POSITION_TYPE setType;
    private boolean isJank;
    private RoutineConstants.POSITION_TYPE type;

    public MoveBotTo(RoutineConstants.POSITION_TYPE type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = false;
        this.type = type;
        // initControllers();
        this.setType = type;
        isRunning = false;
    }

    public MoveBotTo(Pose2d type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = true;
        this.setpoint = type;
        isRunning = false;

        // this.setType = type;
    }

    @Override
    public void initialize() {
        isRunning = true;
        if (!isJank) {
            this.setpoint = PositionState.getPositionPose(type);
        }
        initControllers();
        timer.reset();
        timer.start();
        xController.setGoal(setpoint.getX());
        yController.setGoal(setpoint.getY());
        thetaController.setGoal(setpoint.getRotation().getRadians());
    }

    @Override
    public void execute() {
        measurement = swerve.getPose();
        double xCalc = xController.calculate(measurement.getX());
        double yCalc = yController.calculate(measurement.getY());
        double thetaCalc = thetaController.calculate(measurement.getRotation().getRadians());
        SwerveModuleState[] states;
        if (
            Math.abs(measurement.getRotation().minus(setpoint.getRotation()).getRadians()) < RoutineConstants.ROT_THRESH_RAD
        ) {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                new ChassisSpeeds(
                    xCalc,
                    yCalc,
                    thetaCalc
                )
            );
            runningSpeed = Math.hypot(xCalc, yCalc);
        } else {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                new ChassisSpeeds(
                    0, 0, 
                    thetaCalc
                )
            );
            runningSpeed = 0;
            xController.reset(measurement.getX());
            yController.reset(measurement.getY());
        }

        swerve.setModules(states);
        // Logger.post("calc", xCalc);
        // Logger.post("error", xController.getPositionError());
        // Logger.post("setpoint", xController.getSetpoint().position);
        Logger.post("setPoint", setpoint.toString());
        Logger.post("measurement", measurement.toString());
    }

    @Override
    public boolean isFinished() {
        if (
            (Math.abs(xController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal())) && 
            (Math.abs(yController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal())) && 
            (Math.abs(thetaController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal()))
        ) {
            lastPositionType = this.setType;
            return true;
        } else if (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D) || RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        isRunning = false;
        runningSpeed = 0;
        CommandScheduler.getInstance().schedule(new TurnAndTranslate(measurement.getRotation().getRadians(), measurement.getRotation().getRadians(), 1, 0.25));
    }

    private void initControllers() {
        xController = new ProfiledPIDController(
            RoutineConstants.TRANSLATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );
        xController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);
        xController.reset(swerve.getPose().getX());

        yController = new ProfiledPIDController(
            RoutineConstants.TRANSLATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S,
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );
        yController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);
        yController.reset(swerve.getPose().getY());

        thetaController = new ProfiledPIDController(
            RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, 
                RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S
            )
        );
        thetaController.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.reset(swerve.getPose().getRotation().getRadians());
    }

    public static RoutineConstants.POSITION_TYPE getLastPositionType() {
        return lastPositionType;
    }
}
