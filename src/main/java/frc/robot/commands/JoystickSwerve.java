package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.ExtraLight;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private CommandXboxController joy;
    private CommandJoystick operatorJoy;
    private SwerveDrive swerve;
    private SlewRateLimiter translationRateLimiterX, translationRateLimiterY, translationRateLimiterYSlower, rotationRateLimiter;
    private ProfiledPIDController rotPID;
    private ExtraLight tapeLight;

    public JoystickSwerve () {
        joy = RobotContainer.getController();
        operatorJoy = RobotContainer.getOperatorJoy1();
        swerve = RobotContainer.getSwerve();
        tapeLight = RobotContainer.getLimelightTape();
        addRequirements(swerve);
        translationRateLimiterX = new SlewRateLimiter(JoystickConstants.MAX_ACCEL_TELEOP_M_S_S);
        translationRateLimiterY = new SlewRateLimiter(JoystickConstants.MAX_ACCEL_TELEOP_M_S_S);
        translationRateLimiterYSlower = new SlewRateLimiter(JoystickConstants.MAX_ACCEL_TELEOP_M_S_S/2);
        rotationRateLimiter = new SlewRateLimiter(JoystickConstants.MAX_ANGULAR_ACCEL_TELEOP_RAD_S_S);
        rotPID = new ProfiledPIDController(RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S)
        );
    }

    @Override
    public void initialize() {
        translationRateLimiterX.reset(0);
        translationRateLimiterY.reset(0);
        rotationRateLimiter.reset(0);
        rotPID.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.reset(swerve.getPose().getRotation().getRadians());
        rotPID.setGoal(OdometryMath2023.isBlue() ? 0 : Math.PI);
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

    private double rotationCalc(double joyRotSpeed, boolean usePID) {
        if (OdometryMath2023.onScoringSide()) {
            rotPID.setGoal(OdometryMath2023.isBlue() ? Math.PI : 0);
        } else {
            rotPID.setGoal(OdometryMath2023.isBlue() ? 0 : Math.PI);
        }
        if (!usePID) {
            rotPID.reset(swerve.getRotation2d().getRadians());
            return joyRotSpeed;
        } else {
            return rotPID.calculate(swerve.getRotation2d().getRadians());
        }
    }

    private SwerveModuleState[] joystickCalculations(CommandXboxController joy) {
        double[] unitsMultiplier = getMultiplier(joy);
        double xSpeed = -joy.getRightX();
        double ySpeed = -joy.getRightY();
        double x2Speed = -joy.getLeftX();
        x2Speed = Math.copySign(Math.pow(x2Speed, JoystickConstants.CONTROLLER_TURNING_EXPONENT), x2Speed);
        
        xSpeed = Math.abs(xSpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > (JoystickConstants.CONTROLLER_DEADBAND) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (JoystickConstants.CONTROLLER_DEADBAND) ? x2Speed : 0;

        if (joy.getHID().getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - joy.getHID().getPOV())) * .05;
            xSpeed = Math.sin(Math.toRadians(360 - joy.getHID().getPOV())) * .05;
        }

        if ((RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_SUBSTATION)) && !OdometryMath2023.onScoringSide()) {
            if (RobotContainer.getClaw().proxCovered()) {
                ySpeed = 0;
            } else {
                ySpeed = translationRateLimiterYSlower.calculate(SwerveConstants.SS_SPEED_M_S);
            }
            translationRateLimiterY.reset(ySpeed);
        } else {
            ySpeed = translationRateLimiterY.calculate(ySpeed * unitsMultiplier[0]);
            translationRateLimiterYSlower.reset(ySpeed);
        }

        xSpeed = translationRateLimiterX.calculate(xSpeed * unitsMultiplier[0]);
        x2Speed = rotationRateLimiter.calculate(x2Speed * unitsMultiplier[1]);

        Rotation2d rot;
        if (OdometryMath2023.isBlue()) {
            rot = swerve.getRotation2d();
        } else {
            rot = swerve.getRotation2d().plus(new Rotation2d(Math.PI));
        }

        if (
            OdometryMath2023.inCommunity() && 
            swerve.getFieldRelative() && 
            OdometryMath2023.facingForward(15) &&
            joy.getHID().getLeftBumper()) {
            x2Speed = RobotContainer.getLimelightTape().getAutoAlignCalc();
        }
        
        ChassisSpeeds chassisSpeeds = 
            swerve.getFieldRelative() ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    ySpeed, 
                    xSpeed, 
                    rotationCalc(
                        x2Speed, 
                        RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_SUBSTATION)
                        || (OdometryMath2023.inCommunity() && (!joy.getHID().getLeftBumper() || (joy.getHID().getLeftBumper() && !OdometryMath2023.facingForward(15))) && (!joy.getHID().getRightBumper()) && (RobotContainer.getClaw().proxCovered()) && (Math.hypot(xSpeed, ySpeed) != 0) && (DriverStation.isTeleopEnabled()))),
                    rot)
            : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);

        SwerveModuleState[] states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }   
    
    //[translationSpeed, rotationSpeed]
    private double[] getMultiplier(CommandXboxController joy) {
        if (joy.getHID().getLeftBumper()) {
            if (RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_SUBSTATION)){
                return new double[] {1.5, 1.5};
            } else {
                return new double[] {JoystickConstants.DAMPEN_SPEED_M_S, JoystickConstants.DAMPEN_ANGULAR_SPEED_RAD_S};
            }
        } else {
            return new double[] {JoystickConstants.REGULAR_SPEED_M_S, JoystickConstants.REGULAR_ANGULAR_SPEED_RAD_S};
        }
    }

}
