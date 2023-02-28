package frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class SwerveDrive extends SubsystemBase {

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private WPI_Pigeon2 gyro;
    private boolean isFieldRelative;
    private boolean locked;
    private SwerveDriveOdometry odometer;
    private Field2d field;

    public SwerveDrive (WPI_Pigeon2 m_gyro) {
        this.frontLeft = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_RIGHT);
        this.gyro = m_gyro;
        isFieldRelative = MiscConstants.FIELD_RELATIVE_ON_START;
        locked = false;
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

    public double getRollDeg() {
        return gyro.getRoll();
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

    public void setModules(SwerveModuleState[] desiredStates) {
        if (!locked) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC); //dampens a little
            frontLeft.setModState(desiredStates[0]);
            frontRight.setModState(desiredStates[1]);
            backLeft.setModState(desiredStates[2]);
            backRight.setModState(desiredStates[3]);
        } else {
            frontLeft.setModState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRight.setModState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backLeft.setModState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backRight.setModState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        Logger.post("reset pose", pose.toString());
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void updateVision(Pose2d pose) {
        getEstimator().resetPosition(getRotation2d(), getModulePositions(), new Pose2d(pose.getX(), pose.getY(), getRotation2d()));
    }

    public void setHeading(double deg) {
        gyro.setYaw(deg);
    }

    public void resetMods() {
        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < 4; i++) {
            modules[i].getTurnSpark().getEncoder().setPosition(modules[i].getAbsEncRad());
        }
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(getPose());
        log();
    }

    public List<SwerveModule> getModules() {
        List<SwerveModule> list = List.of(frontLeft, frontRight, backLeft, backRight);
        return list;
    }

    public SwerveDriveOdometry getEstimator() {
        return odometer;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition ret[] = {frontLeft.getModPosition(), frontRight.getModPosition(), backLeft.getModPosition(), backRight.getModPosition()};
        
        return ret;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState ret[] = {frontLeft.getModState(), frontRight.getModState(), backLeft.getModState(), backRight.getModState()};
        
        return ret;
    }

    public double[] getAccelerationMPS2() {
        short[] accelerometerValues = new short[3];
        double[] mag = new double[3];
        gyro.getBiasedAccelerometer(accelerometerValues);
        for (int i = 0; i < 3; i++) {
            mag[i] = (accelerometerValues[i] / 16384.0) * 9.8;
        }
        double hyp = Math.hypot(mag[0], mag[1]);
        double [] ret = {hyp, OdometryMath2023.smartArcAngle(mag[0], mag[1], hyp)};
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

    public boolean getLocked() {
        return locked;
    }

    public void setLocked(boolean lock) {
        this.locked = lock;
    }
    
    private void log() {
        Logger.post("FieldRelative", getFieldRelative());
        // // Logger.post("GyroCalibrating", gyro.isCalibrating());
        // Logger.post("odom", getPose().toString());
        // // Logger.post("estimator pose", poseEstimator.getEstimatedPosition());
        // // Logger.post("key", backLeft.getTurnPosRad());
        // Logger.post("gyro", getHeading());

        // Logger.post("backLeft", backLeft.getErrors());
        // Logger.post("frontLeft", frontLeft.getErrors());
        // Logger.post("backRight", backRight.getErrors());
        // Logger.post("frontRight", frontRight.getErrors());
        Logger.postComplex("Field5427", field);

        Logger.post("accerlation magnitude", getAccelerationMPS2()[0]);
        Logger.post("accerlation direction", getAccelerationMPS2()[1]);
        // Logger.post("abs FR", frontRight.getAbsEncRaw());
        // Logger.post("abs FL", frontLeft.getAbsEncRaw());
        // Logger.post("abs BR", backRight.getAbsEncRaw());
        // Logger.post("abs BL", backLeft.getAbsEncRaw());

        // Logger.post("abs rad FR", frontRight.getAbsEncRad());
        // Logger.post("abs rad FL", frontLeft.getAbsEncRad());
        // Logger.post("abs rad BR", backRight.getAbsEncRad());
        // Logger.post("abs rad BL", backLeft.getAbsEncRad());

        // Logger.post("frontLeft.getModPosition()", frontLeft.getModPosition().toString());
        // Logger.post("frontRight.getModPosition()", frontRight.getModPosition().toString());
        // Logger.post("bakcLeft.getModPosition()", backLeft.getModPosition().toString());
        // Logger.post("backRight.getModPosition()", backRight.getModPosition().toString());
        // Logger.post("speeds", frontRight.getDriveSpeed());

        // Logger.post("speed RPM", frontRight.backToRPM());

        // Logger.post("state", frontRight.getModState().toString());

        // // Logger.post("gyro yaw", OdometryMath2022.gyroTargetOffset());
        // Logger.post("x2speed", x2Speed);
        // Logger.post("usingOdom", usingOdometryTargeting);   
        
        // SmartDashboard.putString("AprilTag Info", aprilTagPi.getTarget().toString());
        // SmartDashboard.putBoolean("has tag", aprilTagPi.hasTarget());

        Logger.post("drivetrain Pitch", getPitchDeg());
        Logger.post("drivetrain Roll", getRollDeg());

    }
}
