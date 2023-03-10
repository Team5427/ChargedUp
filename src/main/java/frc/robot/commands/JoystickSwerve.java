package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private XboxController joy;
    private SwerveDrive swerve;
    private AHRS gyro;

    public JoystickSwerve () {
        joy = RobotContainer.getController();
        swerve = RobotContainer.getSwerve();
        gyro = RobotContainer.getAHRS();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy.getAButtonPressed()) {swerve.toggleFieldRelative();}
        if (joy.getBButtonPressed()) {
            gyro.zeroYaw();
            swerve.resetOdometry(new Pose2d(5.93, 3.84, new Rotation2d(0)));
            swerve.resetMods();
        }

        states = swerve.controllerToModuleStates(joy);
        
        swerve.setModules(states);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMods();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    

}
