package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveRobotOriented extends CommandBase {

    private double speed;
    private double time;
    private Rotation2d rot;
    private ChassisSpeeds cSpeeds;
    private Timer timer;
    private SwerveDrive dt;

    public MoveRobotOriented(double speed, double time, Rotation2d rot) {
        this.speed = speed;
        this.time = time;
        this.rot = rot;
        timer = new Timer();
        dt = RobotContainer.getSwerve();
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        cSpeeds = new ChassisSpeeds(speed * rot.getCos(), speed * rot.getSin(), 0);
    }

    @Override
    public void execute() {
        dt.setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(cSpeeds));
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > time) || (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D));
    }
    
}
