package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private Joystick joy;
    private SwerveDrive swerve;
    private WPI_Pigeon2 gyro;

    public JoystickSwerve () {
        joy = RobotContainer.getJoy();
        swerve = RobotContainer.getSwerve();
        gyro = RobotContainer.getPigeon();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy.getAButtonPressed()) {swerve.toggleFieldRelative();}
        if (joy.getBButtonPressed()) {
            gyro.setYaw(0);
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
