package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class TurnAndTranslate extends CommandBase {

    private SwerveDrive dt;
    private Rotation2d headingRadians;
    private Rotation2d holonomicRotationRadians;
    private double speedMPS;
    private ProfiledPIDController headingPID;
    private double time;
    private boolean timed;
    private Timer timer;

    public TurnAndTranslate(Rotation2d headingRadians, Rotation2d holonomicRotationRadians, double speedMPS, double time) {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        this.headingRadians = headingRadians;
        this.holonomicRotationRadians = holonomicRotationRadians;
        this.speedMPS = speedMPS;
        this.time = time;
        this.timed = true;
        timer = new Timer();
    }

    public TurnAndTranslate(Rotation2d headingRadians, Rotation2d holonomicRotationRadians, double speedMPS) {
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

        if (!OdometryMath2023.isBlue()) {
            this.headingRadians = OdometryMath2023.flip(this.headingRadians);
            this.holonomicRotationRadians = OdometryMath2023.flip(this.holonomicRotationRadians);
        }

        Rotation2d rot = dt.getRotation2d();

        headingPID.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setGoal(headingRadians.getRadians());
        headingPID.reset(rot.getRadians());
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        Rotation2d rot = dt.getRotation2d();
        double calc = headingPID.calculate(rot.getRadians());
        SwerveModuleState[] moduleStates;

        moduleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speedMPS * holonomicRotationRadians.getCos(),
                speedMPS * holonomicRotationRadians.getSin(),
                calc,
                rot
            )
        );
        dt.setModules(moduleStates);
    }

    @Override
    public boolean isFinished() {
        if (RobotContainer.getJoy().getHID().getRawButton(8)) {
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
