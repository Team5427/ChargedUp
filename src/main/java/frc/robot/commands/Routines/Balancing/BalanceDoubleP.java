package frc.robot.commands.Routines.Balancing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class BalanceDoubleP extends CommandBase {

    private SwerveDrive dt;
    private PIDController weakerPID;
    private PIDController strongerPID;
    private Timer timer;
    private static final double WEAKER_P = 2; //FIXME may need to negate
    private static final double STRONGER_P = 2; //FIXME may need to negate
    private static final double SWITCH_THRESHHOLD_DEG = 5; //FIXME
    private static final double ERROR_THRESHHOLD_DEG = 2; //FIXME
    private static final double TIME_AT_GOAL_SEC = 2;

    public BalanceDoubleP() {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        weakerPID = new PIDController(WEAKER_P, 0, 0);
        strongerPID = new PIDController(STRONGER_P, 0, 0);
        weakerPID.setSetpoint(0);
        strongerPID.setSetpoint(0);
        timer.reset();
    }

    @Override
    public void execute() {
        double measurement = dt.getRobotTiltGlobalYAxisDeg();
        double calc = Math.abs(measurement) < SWITCH_THRESHHOLD_DEG ? 
            weakerPID.calculate(measurement) :
            strongerPID.calculate(measurement);

        // if (!OdometryMath2023.isBlue()) {
        //     calc *= -1;
        // } //prolly dont need this but brain dont work at night

        dt.setModules(
            SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                calc, 
                0.0, 
                0.0,
                dt.getRotation2d()
            ))
        );
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(dt.getRobotTiltGlobalYAxisDeg()) < ERROR_THRESHHOLD_DEG) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }

        if (timer.get() > TIME_AT_GOAL_SEC) {
            return true;
        } else {
            return false;
        }
    }
}
