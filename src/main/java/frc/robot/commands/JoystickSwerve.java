package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private CommandJoystick joy;
    private SwerveDrive swerve;
    private SlewRateLimiter translationRateLimiterX, translationRateLimiterY, rotationRateLimiter;

    public JoystickSwerve () {
        joy = RobotContainer.getJoy();
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        translationRateLimiterX = new SlewRateLimiter(JoystickConstants.MAX_ACCEL_TELEOP_M_S_S);
        translationRateLimiterY = new SlewRateLimiter(JoystickConstants.MAX_ACCEL_TELEOP_M_S_S);
        rotationRateLimiter = new SlewRateLimiter(JoystickConstants.MAX_ANGULAR_ACCEL_TELEOP_RAD_S_S);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy.getHID().getRawButton(JoystickConstants.TOGGLE_FIELD_RELATIVE_BUTTON)) {swerve.toggleFieldRelative();}
        if (joy.getHID().getRawButton(JoystickConstants.RESET_ODOMETRY_BUTTON)) {
            swerve.setHeading(0);
            Pose2d resetPose = MiscConstants.DEBUG_RESET_POSE;
            swerve.resetOdometry(OdometryMath2023.isBlue() ? resetPose : OdometryMath2023.flip(resetPose));
            swerve.resetMods();
        }

        states = joystickCalculations(joy);
        
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

    private SwerveModuleState[] joystickCalculations(CommandJoystick joy) {
        double dampener = joy.getHID().getRawAxis(4) * ((1 - JoystickConstants.DAMPENED_SPEED) / 1.0) + JoystickConstants.DAMPENED_SPEED;
        double xSpeed = -joy.getHID().getX();
        double ySpeed = -joy.getHID().getY();
        double x2Speed = -joy.getHID().getZ();
        x2Speed = Math.copySign(Math.pow(x2Speed, JoystickConstants.CONTROLLER_TURNING_EXPONENT), x2Speed);
        
        xSpeed = Math.abs(xSpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? xSpeed : 0; //apply deadband with dampener
        ySpeed = Math.abs(ySpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (JoystickConstants.CONTROLLER_DEADBAND) ? x2Speed : 0;

        xSpeed = translationRateLimiterX.calculate(xSpeed * SwerveConstants.MAX_SPEED_TELEOP_M_PER_S); //apply slew + scale to m/s and rad/s
        ySpeed = translationRateLimiterY.calculate(ySpeed * SwerveConstants.MAX_SPEED_TELEOP_M_PER_S);
        x2Speed = rotationRateLimiter.calculate(x2Speed * SwerveConstants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S);

        xSpeed *= dampener;
        ySpeed *= dampener;
        x2Speed *= dampener;

        if (joy.getHID().getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - joy.getHID().getPOV())) * dampener;
            xSpeed = Math.sin(Math.toRadians(360 - joy.getHID().getPOV())) * dampener;
        }

        // System.out.println("xSpeed:" + xSpeed + "  ySpeed: " + ySpeed + "  x2Speed" + x2Speed);

        ChassisSpeeds chassisSpeeds = swerve.getFieldRelative() ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, swerve.getPose().getRotation()) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);

        SwerveModuleState[] states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }    

}
