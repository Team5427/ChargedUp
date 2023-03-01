package frc.robot.commands.Auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class Balance extends CommandBase {

    private SwerveDrive dt;
    private Timer timer;
    private ChassisSpeeds speeds = new ChassisSpeeds(RoutineConstants.BALANCE_ACTIVATION_SPEED_M_S, 0, 0);
    private ChassisSpeeds speeds2 = new ChassisSpeeds(-RoutineConstants.BALANCE_ACTIVATION_SPEED_M_S, 0, 0);

    public Balance() {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (dt.getPitchDeg() < -RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
        } else if (dt.getPitchDeg() > RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds2));
        } else {
            dt.stopMods();
        }
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D);
    }
}
