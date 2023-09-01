package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class TurnInPlace extends CommandBase {

    private SwerveDrive dt;
    private Rotation2d setPoint;
    private ProfiledPIDController controller;

    public TurnInPlace(Rotation2d setPoint) {
        this.dt = RobotContainer.getSwerve();
        this.setPoint = setPoint;
        // controller = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
        //     new Constraints(RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S));
        controller = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(Math.PI, Math.PI * 1.5));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(3));
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        Timer.delay(0.1);

        SmartDashboard.putBoolean("End", false);

        if (!OdometryMath2023.isBlue()) {
            setPoint = OdometryMath2023.flip(setPoint);
        }
        SmartDashboard.putNumber("Setpoint", setPoint.getDegrees());
        controller.reset(dt.getPose().getRotation().getRadians());
        // controller.reset(0.0);
        controller.setGoal(setPoint.getRadians());
        
    }

    @Override
    public void execute() {
        double measurement = dt.getPose().getRotation().getRadians();
        double calc  = controller.calculate(measurement);
        SwerveModuleState[] states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, calc));
        dt.setModules(states);
        SmartDashboard.putNumber("Error", controller.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return controller.getGoal().equals(controller.getSetpoint()) 
        && Math.abs(controller.getPositionError()) < RoutineConstants.ROTATION_TOLERANCE_RAD;
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopMods();
        SmartDashboard.putBoolean("End", true);
    }
}
