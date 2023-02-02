package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveBotTo extends CommandBase {

    private Pose2d setpoint;
    private Pose2d measurement;
    private SwerveDrive swerve;
    private static ProfiledPIDController xController, yController, thetaController;
    private Timer timer;

    public MoveBotTo(Pose2d setpoint) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        this.setpoint = setpoint;
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
        SwerveModuleState[] states;
        if (
            Math.abs(measurement.getRotation().minus(setpoint.getRotation()).getRadians()) > Constants.RoutineConstants.ROUTINE_THRESHOLD_ROT_ERROR_RAD
        ) {
            states = Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                calculateClosedLoop(measurement, setpoint)
            );
        } else {
            states = Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                new ChassisSpeeds(
                    0, 0, 
                    thetaController.calculate(
                        measurement.getRotation().getRadians(), 
                        setpoint.getRotation().getRadians()
                    )
                )
            );
        }

        swerve.setModules(states);
    }

    @Override
    public boolean isFinished() {
        if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
            return true;
        } else if (RobotContainer.getJoy().getHID().getRawButton(2)) {
            return true;
        } else {
            return false;
        }
    }

    private ChassisSpeeds calculateClosedLoop(Pose2d current, Pose2d setpoint) {
        double xSpeed = xController.calculate(current.getX(), setpoint.getX());
        double ySpeed = yController.calculate(current.getY(), setpoint.getY());
        double rotSpeed = thetaController.calculate(current.getRotation().getRadians(), setpoint.getRotation().getRadians());
        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, current.getRotation());
    }

    private void initControllers() {
        xController = new ProfiledPIDController(
            0, 0, 0, 
            new Constraints(
                Constants.RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                Constants.RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );

        yController = new ProfiledPIDController(
            0, 0, 0, 
            new Constraints(
                Constants.RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                Constants.RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );

        thetaController = new ProfiledPIDController(
            0, 0, 0, 
            new Constraints(
                Constants.RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, 
                Constants.RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S
            )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
}
