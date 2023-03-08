package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private CommandJoystick operatorJoy;
    private SwerveDrive swerve;
    private SlewRateLimiter translationRateLimiterX, translationRateLimiterY, rotationRateLimiter;

    public JoystickSwerve () {
        joy = RobotContainer.getJoy();
        operatorJoy = RobotContainer.getOperatorJoy1();
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
        double[] unitsMultiplier = getMultiplier(joy);
        double xSpeed = -joy.getHID().getX();
        double ySpeed = -joy.getHID().getY();
        double x2Speed = -joy.getHID().getZ();
        x2Speed = Math.copySign(Math.pow(x2Speed, JoystickConstants.CONTROLLER_TURNING_EXPONENT), x2Speed);
        
        xSpeed = Math.abs(xSpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (JoystickConstants.CONTROLLER_DEADBAND) ? x2Speed : 0;

        if (joy.getHID().getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - joy.getHID().getPOV())) * .05;
            xSpeed = Math.sin(Math.toRadians(360 - joy.getHID().getPOV())) * -.05;
        }

        // ALLOW OPERATOR POV IF DRIVER NOT DRIVING
        if(x2Speed == 0 && ySpeed == 0 && xSpeed == 0){
            if (operatorJoy.getHID().getPOV() != -1) {
                ySpeed = Math.cos(Math.toRadians(360 - operatorJoy.getHID().getPOV())) * .1;
                xSpeed = Math.sin(Math.toRadians(360 - operatorJoy.getHID().getPOV())) * .1;
            }
        }

        
        xSpeed = translationRateLimiterX.calculate(xSpeed * unitsMultiplier[0]);
        ySpeed = translationRateLimiterY.calculate(ySpeed * unitsMultiplier[0]);
        x2Speed = rotationRateLimiter.calculate(x2Speed * unitsMultiplier[1]);

        Rotation2d rot;
        if (OdometryMath2023.isBlue()) {
            rot = swerve.getRotation2d();
        } else {
            rot = swerve.getRotation2d().plus(new Rotation2d(Math.PI));
        }
        
        ChassisSpeeds chassisSpeeds = swerve.getFieldRelative() ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, rot) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);

        SwerveModuleState[] states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }   
    
    //[translationSpeed, rotationSpeed]
    private double[] getMultiplier(CommandJoystick joy) {
        if (joy.getHID().getRawButton(JoystickConstants.DAMPEN)) {
            return new double[] {JoystickConstants.DAMPEN_SPEED_M_S, JoystickConstants.DAMPEN_ANGULAR_SPEED_RAD_S};
        } else {
            return new double[] {JoystickConstants.REGULAR_SPEED_M_S, JoystickConstants.REGULAR_ANGULAR_SPEED_RAD_S};
        }
    }

}
