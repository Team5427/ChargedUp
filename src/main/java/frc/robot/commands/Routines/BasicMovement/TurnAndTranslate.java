package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TurnAndTranslate extends CommandBase {

    private SwerveDrive dt;
    private double headingRadians;
    private double holonomicRotationRadians;
    private double speedMPS;
    private ProfiledPIDController headingPID;
    private double time;
    private boolean timed;
    private Timer timer;

    public TurnAndTranslate(double headingRadians, double holonomicRotationRadians, double speedMPS, double time) {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        this.headingRadians = headingRadians;
        this.holonomicRotationRadians = holonomicRotationRadians;
        this.speedMPS = speedMPS;
        this.time = time;
        this.timed = true;
        timer = new Timer();
    }

    public TurnAndTranslate(double headingRadians, double holonomicRotationRadians, double speedMPS) {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        this.headingRadians = headingRadians;
        this.holonomicRotationRadians = holonomicRotationRadians;
        this.speedMPS = speedMPS;
        this.timed = false;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        headingPID = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S)
        );

        headingPID.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.reset(dt.getPose().getRotation().getRadians());
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        double calc = headingPID.calculate(dt.getPose().getRotation().getRadians());
        SwerveModuleState[] moduleStates;
        moduleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speedMPS * Math.cos(holonomicRotationRadians),
                speedMPS * Math.sin(holonomicRotationRadians),
                calc,
                dt.getPose().getRotation()
            )
        );
        dt.setModules(moduleStates);
    }

    @Override
    public boolean isFinished() {
        if (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D)) {
            return true;
        } else if (timer.get() > time && timed) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

}
