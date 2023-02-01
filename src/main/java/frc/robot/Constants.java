// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {

        // Robot Ports
        public static final int FRONT_LEFT_SPEED_MOTOR = 10; //FIXME
        public static final int FRONT_RIGHT_SPEED_MOTOR = 6; //FIXME
        public static final int BACK_LEFT_SPEED_MOTOR = 3; //FIXME
        public static final int BACK_RIGHT_SPEED_MOTOR = 8; //FIXME

        public static final int FRONT_LEFT_TURN_MOTOR = 2; //FIXME
        public static final int FRONT_RIGHT_TURN_MOTOR = 5; //FIXME
        public static final int BACK_LEFT_TURN_MOTOR = 7; //FIXME
        public static final int BACK_RIGHT_TURN_MOTOR = 9; //FIXME

        public static final int FRONT_LEFT_CANCODER = 11; //FIXME
        public static final int FRONT_RIGHT_CANCODER = 14; //FIXME
        public static final int BACK_LEFT_CANCODER = 13; //FIXME
        public static final int BACK_RIGHT_CANCODER = 12; //FIXME

        public static final double FRONT_LEFT_OFFSET = -2.39; //FIXME
        public static final double FRONT_RIGHT_OFFSET = 3.03; //FIXME
        public static final double BACK_LEFT_OFFSET = .664; //FIXME
        public static final double BACK_RIGHT_OFFSET = -.9; //FIXME

        // Inversions
        public static final boolean FRONT_LEFT_TURNING_INVERT = false; //might change but shouldnt vvv
        public static final boolean FRONT_RIGHT_TURNING_INVERT = false;
        public static final boolean BACK_LEFT_TURNING_INVERT = false;
        public static final boolean BACK_RIGHT_TURNING_INVERT = false;

        public static final boolean FRONT_LEFT_DRIVE_INVERT = true;
        public static final boolean FRONT_RIGHT_DRIVE_INVERT = false;
        public static final boolean BACK_LEFT_DRIVE_INVERT = true;
        public static final boolean BACK_RIGHT_DRIVE_INVERT = false;

        public static final boolean FRONT_LEFT_CANCODER_INVERT = false;
        public static final boolean FRONT_RIGHT_CANCODER_INVERT = false;
        public static final boolean BACK_LEFT_CANCODER_INVERT = false;
        public static final boolean BACK_RIGHT_CANCODER_INVERT = false; //^^^

        // Robot Physical Dimensions
        public static final double DT_WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.83);
        public static final double DT_TRACKWIDTH = Units.inchesToMeters(19.5);
        public static final double DT_WHEELBASE = Units.inchesToMeters(19.5);

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(DT_WHEELBASE / 2, DT_TRACKWIDTH / 2),
                new Translation2d(DT_WHEELBASE / 2, -DT_TRACKWIDTH / 2),
                new Translation2d(-DT_WHEELBASE / 2, DT_TRACKWIDTH / 2),
                new Translation2d(-DT_WHEELBASE / 2, -DT_TRACKWIDTH / 2));

        // Swerve Speed Numbers
        public static final double kDriveMotorGearRatio = ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)); // Mk4 L2 SDS
        public static final double kTurningMotorGearRatio = ((15.0 / 32.0) * (10.0 / 60.0));
        public static final double SWERVE_CONVERSION_FACTOR_ROT_TO_METER = (Math.PI * DT_WHEEL_DIAMETER_METERS * kDriveMotorGearRatio);
        public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S = SWERVE_CONVERSION_FACTOR_ROT_TO_METER / 60;
        public static final double SWERVE_CONVERSION_FACTOR_ROT_TO_RAD = 2 * Math.PI * kTurningMotorGearRatio;
        public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S = SWERVE_CONVERSION_FACTOR_ROT_TO_RAD / 60;
        public static final double MAX_PHYSICAL_SPEED_M_PER_SEC = 4.4196; // do not touch, unless switching from L2
        public static final double MAX_SPEED_TELEOP_M_PER_S = 4; //
        public static final double MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S = Math.PI * 2;

        // AUTON STUFF
        public static final double MAX_AUTON_ACCEL_M_PER_S2 = 1;
        public static final double MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2 = Math.PI * 3 * 2;
        public static final double AUTON_TRANSLATION_P = 1.5; // FIXME lmao these r bbq values for rn
        public static final double AUTON_THETA_P = 3; // FIXME
        public static final double MAX_AUTON_SPEED_M_PER_S = 2;

        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S, MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2);

        // CONTROLLER CONSTANTS MODULES
        public static final double TURNING_PID_P = 2.9;
        public static final double TURNING_PID_D = 0.2;
        public static final double TURNING_FF_S = 0.088444;
        public static final double TURNING_FF_V = 0.24913;
        public static final double TURNING_FF_A = 0.011425;
        public static final double TURNING_MAX_SPEED_RAD_S = SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S * MAX_NEO_SPEED_RPM;
        public static final double TURNING_MAX_ACCEL_RAD_S_S = TURNING_MAX_SPEED_RAD_S * 4;

        public static final double TURNING_P = 0.35;

        public static final double SPEED_PID_P = 2.94;
        public static final double SPEED_FF_S = 0.097718;
        public static final double SPEED_FF_V = 2.5872;
        public static final double SPEED_FF_A = 0.22077;
        public static final double SPEED_MAX_SPEED_M_S = SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S * MAX_NEO_SPEED_RPM;
        public static final double SPEED_MAX_ACCEL_M_S_S = SPEED_MAX_SPEED_M_S * 3;

        public static enum SwerveModuleType {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }
    }

    public static final class VisionConstants {
        public static final double LIMELIGHT_MOUNT_ANGLE_RAD = 25; // FIXME angle between back of limelight and front of
        // vertical bar
        public static final double LIMELIGHT_LENS_HEIGHT_M = 25; // FIXME height of lens
        public static final double GOAL_HEIGHT_M = 2.64;
        public static final Translation3d CAMERA_TRANSLATION = new Translation3d(0, 0, 0); // FIXME
        public static final Rotation3d CAMERA_ROTATION = new Rotation3d(0, 0, 0); // FIXME
    }

    public static final class JoystickConstants {
        public static final double CONTROLLER_DEADBAND = 0.1;
        public static final double CONTROLLER_TURNING_EXPONENT = 2.5;
        public static final double MAX_ACCEL_TELEOP_M_S_S = 4.0;
        public static final double MAX_ANGULAR_ACCEL_TELEOP_RAD_S_S = Math.PI;
        public static final double DAMPENED_SPEED = 0.3;
    }

    public static final class RoutineConstants {
        public static final double ROUTINE_MAX_TRANSLATION_SPEED_M_S = 3.0;
        public static final double ROUTINE_MAX_ROTATION_SPEED_RAD_S = Math.PI;
        public static final double ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S = 1.0;
        public static final double ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S = Math.PI;
        public static final double ROUTINE_THRESHOLD_ROT_ERROR_RAD = Math.PI/2;
    }

    public static final class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 2; //FIXME
        public static final int RIGHT_MOTOR_ID = 2; //FIXME
        public static final int LEFT_LIMIT_ID = 2; //FIXME
        public static final int RIGHT_LIMIT_ID = 2; //FIXME
        public static final double GEARBOX_GEARING = (9.0 / 62.0);
        public static final double SPROCKET_PD = Units.inchesToMeters(1.751);
        public static final double POSITION_CONVERSION_FACTOR_ROT_TO_METERS = GEARBOX_GEARING * Math.PI * SPROCKET_PD;
        public static final double VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS = POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 60;
        public static final double LOWER_LIMIT_METERS = .25; //FIXME resting height
        public static final double UPPER_LIMIT_METERS = 1.25; //FIXME just pull up and readout
        public static final double FF_S = .5; //FIXME
        public static final double FF_G = .5; //FIXME
        public static final double FF_V = .5; //FIXME
        public static final double FF_A = .5; //FIXME
        public static final double P = .5; //FIXME
        public static final double I = .5; //FIXME
        public static final double D = .5; //FIXME
        public static final double MAX_SPEED_M_S = MAX_NEO_SPEED_RPM * VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS; //FIXME rn theoretical
        public static final double MAX_ACCEL_M_S_S = MAX_SPEED_M_S / 4; //FIXME theoretical rn
        public static final double GOAL_TOLERANCE_METERS = .01;
        public static final int CURRENT_LIMIT_AMPS = 45;

    }
    // DEBUG VARS (Remove before comp if robot is stable)
    public static final boolean FIELD_RELATIVE_SWITCHABLE = true;
    public static final boolean FIELD_RELATIVE_ON_START = false;

    // DRIVER SETTINGS
    public static final int DRIVER_CONTROLLER = 0;


    // CAN IDS
    public static final int PIGEON_ID = 16;


    // BUTTON ID
    public static final int TOGGLE_FIELD_RELATIVE_BUTTON = 12;
    public static final int RESET_ODOMETRY_BUTTON = 11;
    public static final double MAX_NEO_SPEED_RPM = 5676.0;

}
