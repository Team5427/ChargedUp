package frc.robot.commands.Routines.Balancing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceBinary extends CommandBase {

    private SwerveDrive dt;
    private Timer timer;

    private final double BINARY_MOVEMENT_THRESH_DEG = 10;
    private final double BINARY_MOVEMENT_SPEED_M_S = 0.5;

    public BalanceBinary() {
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

        if (pitch < -BINARY_MOVEMENT_THRESH_DEG) {
            xCalc = BINARY_MOVEMENT_SPEED_M_S;
            timer.stop();
            timer.reset();
        } else if (pitch > BINARY_MOVEMENT_THRESH_DEG) {
            timer.stop();
            timer.reset();
            xCalc = -BINARY_MOVEMENT_SPEED_M_S;
        } else {
            xCalc = 0.0;
            timer.start();
        }

        dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
            xCalc,
            0,
            0
        )));
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O) || (timer.get() > RoutineConstants.BALANCED_TIME);
    }
}
