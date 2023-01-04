package frc.robot.subsystems.Swerve;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagPi;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2022;

public class SwerveDrive extends SubsystemBase {

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private AHRS gyro;
    private double xSpeed;
    private double ySpeed;
    private double x2Speed;
    private SlewRateLimiter xRateLimiter, yRateLimiter, xRateLimiter2, visionLimiter;
    private ChassisSpeeds chassisSpeeds;
    private boolean isFieldRelative;
    private double dampener;
    private PIDController faceTargetPID;
    private SwerveDriveOdometry odometer;
    private Field2d field;
    private boolean usingOdometryTargeting = false;
    // private AprilTagPi aprilTagPi;

    public SwerveDrive (AHRS m_gyro, AprilTagPi pi) {
        this.frontLeft = new SwerveModule(Constants.SwerveConstants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(Constants.SwerveConstants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(Constants.SwerveConstants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(Constants.SwerveConstants.SwerveModuleType.BACK_RIGHT);
        this.gyro = m_gyro;
        // this.aprilTagPi = pi;
        isFieldRelative = Constants.FIELD_RELATIVE_ON_START;
        dampener = 1;
        xRateLimiter = new SlewRateLimiter(Constants.SwerveConstants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        yRateLimiter = new SlewRateLimiter(Constants.SwerveConstants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        xRateLimiter2 = new SlewRateLimiter(Constants.SwerveConstants.MAX_ANGULAR_ACCEL_TELEOP_PERCENT_PER_S);
        visionLimiter = new SlewRateLimiter(Constants.SwerveConstants.MAX_ANGULAR_ACCEL_TELEOP_PERCENT_PER_S * 1.25);
        odometer = new SwerveDriveOdometry(Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS, getRotation2d(), getModulePositions(), getPose());
        faceTargetPID = new PIDController(.065, 0, 0);
        //edit and add to constants //FIXME tune and maybe invert P (if it goes in wrong direction)
        faceTargetPID.setTolerance(5);
        field = new Field2d();
        zeroHeading();
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public void setGyroOffset(double angleDeg) {
        gyro.setAngleAdjustment(angleDeg);
    }

    public double getHeading() {
        return Math.IEEEremainder((360 - gyro.getAngle()), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(Math.IEEEremainder((360 - gyro.getYaw()), 360)); //NOT AFFECTED BY SET ANGLE ADJUSTMENT
        // return 4.0;
    }

    public void stopMods() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModuleState[] controllerToModuleStates(XboxController controller) {
        dampener = ((Constants.SwerveConstants.DAMPENER_LOW_PERCENT - 1) * controller.getLeftTriggerAxis() + 1);
        double shootButton = controller.getRightTriggerAxis();

        xSpeed = -controller.getLeftX() * dampener;
        ySpeed = -controller.getLeftY() * dampener;
        x2Speed = Math.signum(-controller.getRightX()) * Math.pow(Math.abs(controller.getRightX()), Constants.SwerveConstants.CONTROLLER_TURNING_EXPONENT * dampener) * dampener;
        //dampens exponent as well as speed

        xSpeed = Math.abs(xSpeed) > (Constants.SwerveConstants.CONTROLLER_DEADBAND * dampener) ? xSpeed : 0; //apply deadband with dampener
        ySpeed = Math.abs(ySpeed) > (Constants.SwerveConstants.CONTROLLER_DEADBAND * dampener) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (Constants.SwerveConstants.CONTROLLER_DEADBAND * dampener) ? x2Speed : 0;

        xSpeed = xRateLimiter.calculate(xSpeed) * Constants.SwerveConstants.MAX_SPEED_TELEOP_M_PER_S; //apply slew + scale to m/s and rad/s
        ySpeed = yRateLimiter.calculate(ySpeed) * Constants.SwerveConstants.MAX_SPEED_TELEOP_M_PER_S;
        x2Speed = xRateLimiter2.calculate(x2Speed) * Constants.SwerveConstants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S;

        if (controller.getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - controller.getPOV())) * dampener;
            xSpeed = Math.sin(Math.toRadians(360 - controller.getPOV())) * dampener;
        }

        // boolean targetVis = aprilTagPi.hasTarget();


        // if (shootButton > .1) {

        //     if (targetVis) {
        //         usingOdometryTargeting = false;
        //         // double yaw = aprilTagPi.getTarget().getYaw();
        //         double visionSpeed = faceTargetPID.calculate(yaw, 0);
                
        //         visionSpeed = (Math.abs(visionSpeed) > .3)?(Math.copySign(.3, visionSpeed)):visionSpeed; //makes pid stupid near the end
        //         if (faceTargetPID.atSetpoint()) {
        //             resetTargetingPID(yaw, Math.toDegrees(visionSpeed));
        //             addVisionMeasurement(RobotContainer.getPi().getVisionBasedRobotPose());
        //             // setGyroOffset(OdometryMath2022.gyroTargetOffset()); //might need to negate //FIXME
        //         }
        //         if (shootButton > .9) {
        //             x2Speed = visionSpeed;
        //         } else {
        //             // x2Speed = x2Speed * (1 - shootButton) + visionSpeed * shootButton;
        //             x2Speed = visionSpeed; //use this whenever debugging to see what vision speed is
        //         }
        //     } else {
        //         usingOdometryTargeting = true;
        //         x2Speed = visionLimiter.calculate(OdometryMath2022.robotEasiestTurnToTarget()) * Constants.SwerveConstants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S;
            
        //     }
        // }
        chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, getPose().getRotation()) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);
        
        //IF YOU ARE WONDERING WHY YSPEED IS IN XSPEED PARAM OF CHASSIS SPEEDS STOP WHAT YOU ARE DOING AND ASK PRAT.
        //DO NOT FLIP.
        //WILL BREAK SPACE TIME FABRIC.
        //DUK WILL NOT BE PROUD.

        //DO NOT CHANGE ANYTHING HERE WITHOUT ASKING PRAT
        //EVER
        //THIS IS SACRED CODE
        //IT WAS WRITTEN 35000 FEET IN THE AIR TRAVELING AT 582 MILES PER HOUR (Spring break '22 -> flight to newark)
        //AND WAS LATER EDITED 35000 FEET IN THE AIR TRAVELING AT 620 MILES PER HOUR (Thanksgiving break '22 -> flight to chicago)

        SwerveModuleState[] states = Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }

    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC);
        frontLeft.setModState(desiredStates[0]);
        frontRight.setModState(desiredStates[1]);
        backLeft.setModState(desiredStates[2]);
        backRight.setModState(desiredStates[3]);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetMods() {
        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < 4; i++) {
            modules[i].getTurnSpark().getEncoder().setPosition(modules[i].getAbsEncRad());
        }
    }

    private void resetTargetingPID(double limelightValue, double angularSpeed) {
        faceTargetPID.reset();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(odometer.getPoseMeters());
        log();
    }

    public List<SwerveModule> getModules() {
        List<SwerveModule> list = List.of(frontLeft, frontRight, backLeft, backRight);
        return list;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition ret[] = {frontLeft.getModPosition(), frontRight.getModPosition(), backLeft.getModPosition(), backRight.getModPosition()};
        return ret;
    }

    public void setBrake(boolean driveBrake, boolean steerBrake) {
        getModules().forEach((mod) -> mod.setBrake(driveBrake, steerBrake));        
    }

    public boolean getFieldRelative() {
        return isFieldRelative;
    }

    public void toggleFieldRelative() {
        isFieldRelative = Constants.FIELD_RELATIVE_SWITCHABLE ? !isFieldRelative : isFieldRelative;
    }
    
    private void log() {
        Logger.post("FieldRelative", getFieldRelative());
        // Logger.post("GyroCalibrating", gyro.isCalibrating());
        Logger.post("odom", odometer.getPoseMeters().toString());
        // Logger.post("estimator pose", poseEstimator.getEstimatedPosition());
        // Logger.post("key", backLeft.getTurnPosRad());
        Logger.post("gyro", getHeading());

        Logger.post("backLeft", backLeft.getErrors());
        Logger.post("frontLeft", frontLeft.getErrors());
        Logger.post("backRight", backRight.getErrors());
        Logger.post("frontRight", frontRight.getErrors());
        Logger.postComplex("Field5427", field);
        Logger.post("abs FR", frontRight.getAbsEncRaw());
        Logger.post("abs FL", frontLeft.getAbsEncRaw());
        Logger.post("abs BR", backRight.getAbsEncRaw());
        Logger.post("abs BL", backLeft.getAbsEncRaw());

        Logger.post("speeds", frontRight.getDriveSpeed());

        Logger.post("speed RPM", frontRight.backToRPM());

        Logger.post("state", frontRight.getModState().toString());
        // Logger.post("gyro yaw", OdometryMath2022.gyroTargetOffset());
        Logger.post("x2speed", x2Speed);
        Logger.post("usingOdom", usingOdometryTargeting);   
        
        // SmartDashboard.putString("AprilTag Info", aprilTagPi.getTarget().toString());
        // SmartDashboard.putBoolean("has tag", aprilTagPi.hasTarget());

    }
}
