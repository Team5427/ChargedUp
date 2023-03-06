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

        ChassisSpeeds speeds = new ChassisSpeeds(
            Math.copySign((-.007 * (Math.pow((Math.abs(dt.getPitchDeg()) -12),2))) + .75 , -dt.getPitchDeg()),
            0,
            0
        );

        if(Math.abs(dt.getPitchDeg()) > 12 ){
            speeds = new ChassisSpeeds(.75,0,0);
        } 

        if (Math.abs(dt.getPitchDeg()) < -RoutineConstants.BALANCE_ACTIVATION_PITCH_DEG) {
            dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
            timer.stop();
            timer.reset();
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
