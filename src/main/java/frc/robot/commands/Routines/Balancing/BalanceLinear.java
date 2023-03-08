package frc.robot.commands.Routines.Balancing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceLinear extends CommandBase {

    private SwerveDrive dt;
    private Timer timer;
    private final double MULTIPLIER = -0.0725;
    private final double TIMER_TOLERANCE_DEG = 11;

    public BalanceLinear() {
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
        double xCalc;
        double pitch = dt.getPitchDeg();

        if (Math.abs(pitch) < TIMER_TOLERANCE_DEG) {
            xCalc = 0; //SHUT_OFF TOLERANCE
            timer.start();
        } else {
            xCalc = MULTIPLIER * pitch;
            timer.stop();
            timer.reset();
        }

        dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
            xCalc,
            0,
            0
        )));
    }

    @Override
    public boolean isFinished() {
        return (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D)) || (timer.get() > RoutineConstants.BALANCED_TIME);
    }
}
