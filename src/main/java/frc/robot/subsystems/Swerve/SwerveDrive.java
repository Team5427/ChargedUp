package frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class SwerveDrive extends SubsystemBase {

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private WPI_Pigeon2 gyro;
    private boolean isFieldRelative;
    private SwerveDriveOdometry odometer;
    private Field2d field;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter thetaLimiter;

    public SwerveDrive (WPI_Pigeon2 m_gyro) {
        this.frontLeft = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_RIGHT);
        xLimiter = new SlewRateLimiter(RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S);
        yLimiter = new SlewRateLimiter(RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S);
        thetaLimiter = new SlewRateLimiter(RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S);
        resetLimiters();
        this.gyro = m_gyro;
        isFieldRelative = MiscConstants.FIELD_RELATIVE_ON_START;
        odometer = new SwerveDriveOdometry(SwerveConstants.SWERVE_DRIVE_KINEMATICS, getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
        field = new Field2d();
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public double getPitchDeg() {
        return gyro.getPitch();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetLimiters() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        thetaLimiter.reset(0);
    }

    public double getRollDeg() {
        return gyro.getRoll();
    }

    //basically returns how much it has tilted on charge station regardless of what
    //what the yaw is; this way we can go on at an angle and get the same value as if we
    //were straight
    public double getRobotTiltGlobalYAxisDeg() {
        Rotation2d yawThetaRad = getRotation2d();
        Rotation2d pitchThetaRad = Rotation2d.fromDegrees(getPitchDeg());
        Rotation2d rollThetaRad = Rotation2d.fromDegrees(getRollDeg());
        double x_g = yawThetaRad.getCos() * pitchThetaRad.getRadians();
        double y_g = yawThetaRad.getSin() * rollThetaRad.getRadians();
        double sign = Math.max(
                Math.abs(x_g), 
                Math.abs(y_g)
            ) == Math.abs(x_g) ? 
                Math.signum(x_g) : 
                Math.signum(y_g);
        return -Math.toDegrees(Math.hypot(x_g, y_g) * sign);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void stopMods() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC); //dampens a little
        frontLeft.setModState(desiredStates[0]);
        frontRight.setModState(desiredStates[1]);
        backLeft.setModState(desiredStates[2]);
        backRight.setModState(desiredStates[3]);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double thetaSpeed = speeds.omegaRadiansPerSecond;
        ChassisSpeeds setSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        setModules(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(setSpeeds));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void updateVision(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), new Pose2d(pose.getX(), pose.getY(), getRotation2d()));
    }

    public void setHeadingRad(double rad) {
        Rotation2d rot = new Rotation2d(rad);
        if (!OdometryMath2023.isBlue()) {
            rot = OdometryMath2023.flip(rot);
        }
        gyro.reset();
        gyro.setYaw(rot.getDegrees());
    }

    public void setHeadingRaw(double rad) {
        Rotation2d rot = new Rotation2d(rad);
        gyro.reset();
        gyro.setYaw(rot.getDegrees());
    }

    public void resetMods() {
        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < 4; i++) {
            modules[i].getTurnSpark().getEncoder().setPosition(modules[i].getAbsEncRad());
            modules[i].resetInversions();
        }
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(getPose());
        if (RobotContainer.getOperatorJoy2().getHID().getRawButton(JoystickConstants.RAMP_PUSH)) {
            resetMods();
        }
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
        isFieldRelative = MiscConstants.FIELD_RELATIVE_SWITCHABLE ? !isFieldRelative : isFieldRelative;
    }

    public void setFieldRelative(boolean op) {
        isFieldRelative = op;
    }
    private void log() {
        Logger.post("FieldRelative", getFieldRelative());
        Logger.postComplex("Field5427", field);

        // Logger.post("Enc front right", frontRight.getAbsEncRad());
        // Logger.post("Enc front left", frontLeft.getAbsEncRad());
        // Logger.post("Enc back left", backLeft.getAbsEncRad());
        // Logger.post("Enc back right", backRight.getAbsEncRad());
        // Logger.post("frontRight error", frontRight.getTurnPID().getPositionError());
        // Logger.post("frontLeft error", frontLeft.getTurnPID().getPositionError());
        // Logger.post("backRight error", backRight.getTurnPID().getPositionError());
        // Logger.post("backLeft error", backLeft.getTurnPID().getPositionError());

        // Logger.post("charge station angle", getRobotTiltGlobalYAxisDeg());
    }
}
