package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveBotTo extends CommandBase {

    private Pose2d setpoint;
    private Pose2d measurement;
    private SwerveDrive swerve;
    private static ProfiledPIDController xController, yController, thetaController;
    private Timer timer;
    private static RoutineConstants.POSITION_TYPE lastPositionType;
    private RoutineConstants.POSITION_TYPE setType;

    public MoveBotTo(RoutineConstants.POSITION_TYPE type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        this.setpoint = PositionState.getPositionPose(type);
        initControllers();
        this.setType = type;
    }

    public MoveBotTo(Pose2d type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        this.setpoint = type;
        initControllers();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
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
            xController.setGoal(setpoint.getX());
            yController.setGoal(setpoint.getY());
            thetaController.setGoal(setpoint.getRotation().getRadians());
        } else {
            xController.setGoal(measurement.getX());
            yController.setGoal(measurement.getY());
            thetaController.setGoal(setpoint.getRotation().getRadians());
        }

        states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(
                xCalc, 
                yCalc, 
                thetaCalc
            )
        );

        swerve.setModules(states);
    }

    @Override
    public boolean isFinished() {
        if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
            lastPositionType = this.setType;
            return true;
        } else if (
            RobotContainer.getPilotJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_P) || 
            RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O)
        ) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Pose2d pose = swerve.getPose();
        xController.reset(pose.getX());
        yController.reset(pose.getY());
        thetaController.reset(pose.getRotation().getRadians());
    }

    private void initControllers() {
        xController = new ProfiledPIDController(
            0.3, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );
        xController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);

        yController = new ProfiledPIDController(
            0.3, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );
        yController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);

        thetaController = new ProfiledPIDController(
            0.3, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, 
                RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S
            )
        );
        thetaController.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static RoutineConstants.POSITION_TYPE getLastPositionType() {
        return lastPositionType;
    }
}
