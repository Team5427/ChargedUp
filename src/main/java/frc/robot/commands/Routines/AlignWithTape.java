package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.TapeLight;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class AlignWithTape extends CommandBase {

    private SwerveDrive dt;
    private TapeLight light;
    private PIDController aligningPID;
    private ProfiledPIDController thetaController;
    private double thetaSetpoint;

    public AlignWithTape() {
        dt = RobotContainer.getSwerve();
        // light = RobotContainer.getTapeLight();
        aligningPID = new PIDController(RoutineConstants.TRANSLATION_P, 0, 0);
        thetaController = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        if (Math.abs(dt.getRotation2d().getRadians()) > (Math.PI / 2)) {
            thetaSetpoint = Math.PI;
        } else {
            thetaSetpoint = 0;
        }
        aligningPID.reset();
        aligningPID.setTolerance(1);
        thetaController.reset(dt.getRotation2d().getRadians());
    }

    @Override
    public void execute() {

        double alignmentCalc = aligningPID.calculate(light.getHorizontal(), thetaSetpoint);
        double thetaCalc = thetaController.calculate(dt.getRotation2d().getRadians());

        SwerveModuleState[] states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, alignmentCalc, thetaCalc, dt.getRotation2d())
        );

        dt.setModules(states);
    }

    @Override
    public boolean isFinished() {
        return aligningPID.atSetpoint() || RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D);
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopMods();
    }
}
