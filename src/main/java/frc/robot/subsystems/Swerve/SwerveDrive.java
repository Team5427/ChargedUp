package frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class SwerveDrive extends SubsystemBase {

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private WPI_Pigeon2 gyro;
    private boolean isFieldRelative;
    private SwerveDriveOdometry odometer;
    private Field2d field;

    public SwerveDrive (WPI_Pigeon2 m_gyro) {
        this.frontLeft = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(SwerveConstants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(SwerveConstants.SwerveModuleType.BACK_RIGHT);
        this.gyro = m_gyro;
        isFieldRelative = Constants.MiscConstants.FIELD_RELATIVE_ON_START;
        odometer = new SwerveDriveOdometry(SwerveConstants.SWERVE_DRIVE_KINEMATICS, getRotation2d(), getModulePositions());
        field = new Field2d();
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_PHYSICAL_SPEED_M_PER_SEC); //dampens a little
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
        isFieldRelative = Constants.MiscConstants.FIELD_RELATIVE_SWITCHABLE ? !isFieldRelative : isFieldRelative;
    }
    
    private void log() {
        Logger.post("FieldRelative", getFieldRelative());
        Logger.post("Odom", odometer.getPoseMeters().toString());

    }
}
