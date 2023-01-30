package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private Joystick joy;
    private SwerveDrive swerve;
    private Pigeon2 gyro;
    private SlewRateLimiter translationRateLimiter, rotationRateLimiter;

    public JoystickSwerve () {
        joy = RobotContainer.getJoy();
        swerve = RobotContainer.getSwerve();
        gyro = RobotContainer.getPigeon();
        addRequirements(swerve);
        translationRateLimiter = new SlewRateLimiter(Constants.JoystickConstants.MAX_ACCEL_TELEOP_M_S_S);
        rotationRateLimiter = new SlewRateLimiter(Constants.JoystickConstants.MAX_ANGULAR_ACCEL_TELEOP_RAD_S_S);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy.getRawButton(Constants.TOGGLE_FIELD_RELATIVE_BUTTON)) {swerve.toggleFieldRelative();}
        if (joy.getRawButton(Constants.RESET_ODOMETRY_BUTTON)) {
            gyro.setYaw(0);
            swerve.resetOdometry(new Pose2d(5.93, 3.84, new Rotation2d(0)));
            swerve.resetMods();
        }

        states = jostickCalculations(joy);
        
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

    private SwerveModuleState[] jostickCalculations(Joystick joy) {

        double dampener = (!joy.getRawButton(6)) ? 1 : Constants.JoystickConstants.DAMPENED_SPEED;
        double xSpeed = -joy.getY() * dampener;
        double ySpeed = -joy.getX() * dampener;
        double x2Speed = Math.signum(-joy.getZ()) * Math.pow(Math.abs(joy.getZ()), Constants.JoystickConstants.CONTROLLER_TURNING_EXPONENT * dampener) * dampener;
        //dampens exponent as well as speed

        xSpeed = Math.abs(xSpeed) > (Constants.JoystickConstants.CONTROLLER_DEADBAND * dampener) ? xSpeed : 0; //apply deadband with dampener
        ySpeed = Math.abs(ySpeed) > (Constants.JoystickConstants.CONTROLLER_DEADBAND * dampener) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (Constants.JoystickConstants.CONTROLLER_DEADBAND * dampener) ? x2Speed : 0;

        xSpeed = translationRateLimiter.calculate(xSpeed) * Constants.SwerveConstants.MAX_SPEED_TELEOP_M_PER_S; //apply slew + scale to m/s and rad/s
        ySpeed = translationRateLimiter.calculate(ySpeed) * Constants.SwerveConstants.MAX_SPEED_TELEOP_M_PER_S;
        x2Speed = rotationRateLimiter.calculate(x2Speed) * Constants.SwerveConstants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S;

        if (joy.getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - joy.getPOV())) * dampener;
            xSpeed = Math.sin(Math.toRadians(360 - joy.getPOV())) * dampener;
        }

        ChassisSpeeds chassisSpeeds = swerve.getFieldRelative() ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, swerve.getPose().getRotation()) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);

        SwerveModuleState[] states = Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }    

}
