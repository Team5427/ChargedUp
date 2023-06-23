package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class TurnInPlace extends CommandBase {

    private SwerveDrive dt;
    private Rotation2d setPoint;
    private ProfiledPIDController controller;

    public TurnInPlace(Rotation2d setPoint) {
        this.dt = RobotContainer.getSwerve();
        this.setPoint = setPoint;
        controller = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S * 2));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(2), Math.toRadians(1));
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        if (!OdometryMath2023.isBlue()) {
            setPoint = OdometryMath2023.flip(setPoint);
        }
        controller.reset(dt.getPose().getRotation().getRadians());
        controller.setGoal(new State(setPoint.getRadians(), 0));
    }

    @Override
    public void execute() {
        double measurement = dt.getPose().getRotation().getRadians();
        double calc  = controller.calculate(measurement);
        dt.setChassisSpeeds(new ChassisSpeeds(0, 0, calc));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopMods();
    }
}
