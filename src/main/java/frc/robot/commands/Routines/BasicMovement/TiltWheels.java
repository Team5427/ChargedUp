package frc.robot.commands.Routines.BasicMovement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.JoystickSwerve;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TiltWheels extends CommandBase {

    private SwerveDrive dt;
    private double[] wheelAngles;
    private Timer timer;

    public TiltWheels(double[] angles) {
        dt = RobotContainer.getSwerve();
        this.wheelAngles = angles;
        timer = new Timer();
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        dt.stopMods();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            dt.getModules().get(i).hardSetModState(new SwerveModuleState(0, new Rotation2d(wheelAngles[i])));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > RoutineConstants.MODULE_TILT_SPEED;
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopMods();
        if (!DriverStation.isAutonomous()) {
            CommandScheduler.getInstance().schedule(new JoystickSwerve());
        }
    }

}
