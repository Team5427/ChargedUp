package frc.robot.commands.Routines;

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
    private ChassisSpeeds speeds = new ChassisSpeeds(RoutineConstants.BALANCE_ACTIVATION_FAST_SPEED_M_S, 0, 0);
    private ChassisSpeeds speeds2 = new ChassisSpeeds(-RoutineConstants.BALANCE_ACTIVATION_FAST_SPEED_M_S, 0, 0);

    private ChassisSpeeds slowSpeeds = new ChassisSpeeds(RoutineConstants.BALANCE_ACTIVATION_MED_SPEED_M_S, 0, 0);
    private ChassisSpeeds slowSpeeds2 = new ChassisSpeeds(-RoutineConstants.BALANCE_ACTIVATION_MED_SPEED_M_S, 0, 0);
    
    private ChassisSpeeds slowerSpeeds = new ChassisSpeeds(RoutineConstants.BALANCE_ACTIVATION_SLOW_SPEED_M_S, 0, 0);
    private ChassisSpeeds slowerSpeeds2 = new ChassisSpeeds(-RoutineConstants.BALANCE_ACTIVATION_SLOW_SPEED_M_S, 0, 0);

    public Balance() {
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (dt.getPitchDeg() < -RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG1) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
            timer.stop();
            timer.reset();
        } else if (dt.getPitchDeg() > RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG1) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds2));
            timer.stop();
            timer.reset();
        } else if (dt.getPitchDeg() < -RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG2){
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(slowSpeeds));
        } else if (dt.getPitchDeg() > RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG2){
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(slowSpeeds2));
        }
        else if (dt.getPitchDeg() < -RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG3){
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(slowerSpeeds));
        } else if (dt.getPitchDeg() > RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG3){
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(slowerSpeeds2));
        }

        else {
            dt.stopMods();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D)) || (timer.get() > RoutineConstants.BALANCED_TIME);
    }
}
