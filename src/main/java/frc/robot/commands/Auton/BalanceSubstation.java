package frc.robot.commands.Auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceSubstation extends CommandBase {

    private SwerveDrive dt;
    private Timer timer;
    private ChassisSpeeds speeds = new ChassisSpeeds(0.5, 0, 0);
    private ChassisSpeeds speeds2 = new ChassisSpeeds(-0.5, 0, 0);

    public BalanceSubstation() {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (dt.getPitchDeg() < -10.0) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
        } else if (dt.getPitchDeg() > 10) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds2));
        } else {
            dt.stopMods();
        }
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.getJoy().getHID().getRawButton(10);
    }
}
