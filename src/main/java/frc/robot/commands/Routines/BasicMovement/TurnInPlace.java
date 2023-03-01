package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TurnInPlace extends CommandBase {

    private Rotation2d setPoint;
    private SwerveDrive dt;
    private ProfiledPIDController controller;
    
    public TurnInPlace(Rotation2d direction) {
        this.setPoint = direction;
        dt = RobotContainer.getSwerve();
        controller = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, new Constraints(
            RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
    }

    @Override
    public void initialize() {
        controller.reset(dt.getPose().getRotation().getRadians());
        controller.setGoal(setPoint.getRadians());
    }

    @Override
    public void execute() {
        double calc = controller.calculate(dt.getPose().getRotation().getRadians());
        ChassisSpeeds rotSpeeds = new ChassisSpeeds(0, 0, calc);
        dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(rotSpeeds));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal() || RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D);
    }

}
