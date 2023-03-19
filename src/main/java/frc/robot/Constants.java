// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.Routines.StateTypes.ClawState;

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
    //EVERYTHING IN METERS AND RADIANS
    public static final class SwerveConstants {

        // Robot Ports
        public static final int FRONT_LEFT_DRIVE_MOTOR = 3; //FIXME
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 9; //FIXME
        public static final int BACK_LEFT_DRIVE_MOTOR = 5; //FIXME
        public static final int BACK_RIGHT_DRIVE_MOTOR = 7; //FIXME

        public static final int FRONT_LEFT_STEER_MOTOR = 2; //FIXME
        public static final int FRONT_RIGHT_STEER_MOTOR = 8; //FIXME
        public static final int BACK_LEFT_STEER_MOTOR = 4; //FIXME
        public static final int BACK_RIGHT_STEER_MOTOR = 6; //FIXME

        public static final int FRONT_LEFT_CANCODER = 13; //FIXME
        public static final int FRONT_RIGHT_CANCODER = 10; //FIXME
        public static final int BACK_LEFT_CANCODER = 11; //FIXME
        public static final int BACK_RIGHT_CANCODER = 12; //FIXME


        public static final double FRONT_LEFT_OFFSET = 2.192; //FIXME
        public static final double FRONT_RIGHT_OFFSET = -1.5769; //FIXME
        public static final double BACK_LEFT_OFFSET = 2.325; //FIXME
        public static final double BACK_RIGHT_OFFSET = -.8590; //FIXME

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

        public static final int PIGEON_ID = 16;

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
        public static final double MAX_PHYSICAL_SPEED_M_PER_SEC = Units.feetToMeters(14.5); // do not touch, unless switching from L2

        // AUTON STUFF
        public static final double MAX_AUTON_ACCEL_M_PER_S2 = 2;
        public static final double MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2 = Math.PI * 1;
        public static final double AUTON_TRANSLATION_P = 2; // FIXME lmao these r bbq values for rn
        public static final double AUTON_THETA_P = 2; // FIXME
        public static final double MAX_AUTON_SPEED_M_PER_S = 3;
        public static final double MAX_AUTON_ANGULAR_SPEED_RAD_S = Math.PI * 1.75;

        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_AUTON_ANGULAR_SPEED_RAD_S, MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2);

        // CONTROLLER CONSTANTS MODULES
        public static final double TURNING_PID_P = 2.9;
        public static final double TURNING_PID_D = 0.2;
        public static final double TURNING_FF_S = 0.088444;
        public static final double TURNING_FF_V = 0.24913;
        public static final double TURNING_FF_A = 0.011425;
        public static final double TURNING_MAX_SPEED_RAD_S = SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S * MiscConstants.MAX_NEO_SPEED_RPM;
        public static final double TURNING_MAX_ACCEL_RAD_S_S = TURNING_MAX_SPEED_RAD_S * 4;

        public static final double TURNING_P = 0.35;

        public static final double SPEED_PID_P = 2.94;
        public static final double SPEED_FF_S = 0.097718;
        public static final double SPEED_FF_V = 2.5872;
        public static final double SPEED_FF_A = 0.22077;
        public static final double SPEED_MAX_SPEED_M_S = SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S * MiscConstants.MAX_NEO_SPEED_RPM;
        public static final double SPEED_MAX_ACCEL_M_S_S = SPEED_MAX_SPEED_M_S * 3;

        public static enum SwerveModuleType {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }
    }

    public static final class VisionConstants {
        public static final Translation3d RIGHT_CAMERA_TRANSLATION = new Translation3d(0, 0, 0); // FIXME
        public static final Rotation3d RIGHT_CAMERA_ROTATION = new Rotation3d(0, 0, 0); // FIXME
        public static final Translation3d LEFT_CAMERA_TRANSLATION = new Translation3d(0, 0, 0); // FIXME
        public static final Rotation3d LEFT_CAMERA_ROTATION = new Rotation3d(0, 0, 0); // FIXME
    }

    public static final class JoystickConstants {

        public static final double CONTROLLER_DEADBAND = 0.08;
        public static final double CONTROLLER_TURNING_EXPONENT = 2;
        public static final double MAX_ACCEL_TELEOP_M_S_S = 3.5 * 3;
        public static final double MAX_ANGULAR_ACCEL_TELEOP_RAD_S_S = Math.PI * 4;
        public static final double REGULAR_SPEED_M_S = 4.25;
        public static final double REGULAR_ANGULAR_SPEED_RAD_S = Math.PI * 1.5;
        public static final double DAMPEN_SPEED_M_S = 0.75;
        public static final double DAMPEN_ANGULAR_SPEED_RAD_S = Math.PI / 4;

        //Joystick IDs
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATION_CONTROLLER = 1;
        public static final int OPERATION2_CONTROLLER = 2;
        public static final int OPERATION_3_CONTROLLER = 3;

        //DRIVER bindings
        
        public static final int TOGGLE_FIELD_OP = 6;
        public static final int RESET_TELEMETRY = 5;
        public static final int CANCEL_ALL_COMMANDS_D = 10;
        public static final int LOCK_SWERVE = 2;
        public static final int DAMPEN = 3;
        public static final int CLAW_BTN = 1;
        public static final int SUBSTATION_PRESET = 7;
        public static final int SS_CANCEL = 8;
        public static final int CLAW_INTAKE = 11;
        public static final int CLAW_OUTTAKE = 12;
        public static final int CLAW_CLAMP = 4;

        //OPERATION bindings
        // Buttons not in use: NONE
        public static final int CANCEL_ALL_COMMANDS_O = 1;
        public static final int HIGH_CONE_PRESET = 2;
        public static final int MID_CONE_PRESET = 3;
        public static final int HIGH_CUBE_PRESET = 4;
        public static final int MID_CUBE_PRESET = 5;
        public static final int FLIP_COLOR = 6;
        public static final int ELEVATOR_RESET = 7; // BIND
        public static final int FLOOR_INTAKE_PRESET_CUBES = 8;
        public static final int FLOOR_INTAKE_PRESET_CONES = 9;
        public static final int ARM_RESET = 10;

        //OPERATION 2 BINDINGS
        // Buttons not in use: 4, 10
        public static final int LOCK_SWERVE_O = 1;
        public static final int UNLOCK_SWERVE_O = 2;
        public static final int SS = 7;
        public static final int BALANCE_BTN = 8;

        public static final int TOP_LEFT_SCORE = 3; //FIXME
        public static final int TOP_RIGHT_SCORE = 4; //FIXME
        public static final int MID_LEFT_SCORE = 5; //FIXME
        public static final int MID_RIGHT_SCORE = 6; //FIXME

    }

    public static final class RoutineConstants {
        public static final double ROUTINE_MAX_TRANSLATION_SPEED_M_S = 4.0;
        public static final double ROUTINE_MAX_ROTATION_SPEED_RAD_S = Math.PI * 1.5;
        public static final double ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S = 2.5;
        public static final double ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S = Math.PI * 2;

        public static final double ROT_THRESH_RAD = Math.PI/4;
        public static final double SCORING_LEVEL_OFFSET_METERS = Units.inchesToMeters(66);
        public static final double TRANSLATION_TOLERANCE_METERS = 0.025;
        public static final double ROTATION_TOLERANCE_RAD = Units.degreesToRadians(2);

        public static final double MOVE_BOT_TO_REPEAT_THRESHOLD_SEC = 0.25;

        public static final double Y_LEVEL_1_METERS = Units.inchesToMeters(75.34); 
        public static final double Y_LEVEL_2_METERS = Units.inchesToMeters(141.34);
        public static final double Y_LEVEL_3_METERS = Units.inchesToMeters(216.03);

        public static final double TRANSLATION_P = 3;
        public static final double ROTATION_P = 4.5;

        public static final double TAPE_TY_SETPOINT = 0.0; //FIXME

        public static final double BALANCE_ACTIVATION_PITCH_DEG = 8.5;

        public static final double BALANCED_TIME = 2;
        public static final double DEBUG_INTEGRATE_DELAY_TIME = 1;

        public static final double CLOSE_COMMUNITY_X_METERS = 2.25;

        //PRESETS
        public static final Pose2d BOTTOM_CONE_SCORING_POSE_DEFAULT = new Pose2d(1.85, 0.5, new Rotation2d(Math.PI));
        public static final Pose2d CUBE_SCORING_POSE_DEFAULT = new Pose2d(1.85, 1.05, new Rotation2d(Math.PI));
        public static final Pose2d TOP_CONE_SCORING_POSE_DEFAULT = new Pose2d(1.85, 1.63, new Rotation2d(Math.PI));
        public static final Pose2d BOTTOM_SUBSTATION_POSE_DEFAULT = new Pose2d(15.39, 6, new Rotation2d(0));
        public static final Pose2d TOP_SUBSTATION_POSE_DEFAULT = new Pose2d(15.39, 7.5, new Rotation2d(0));

        public static final Pose2d debug = new Pose2d(0, 0, new Rotation2d(0));

        public static enum POSITION_TYPE {
            LEFT_CONE,
            CUBE,
            RIGHT_CONE,
            LEFT_SS,
            RIGHT_SS
        }

        public static final ClawState DEFAULT_CLAW_STATE = new ClawState(0, ArmConstants.UPPER_LIMIT_RAD);
        public static final ClawState TOP_CONE_CLAW_STATE = new ClawState(ElevatorConstants.UPPER_LIMIT_METERS - .02, Units.degreesToRadians(11.85), true);
        public static final ClawState MID_CONE_CLAW_STATE = new ClawState(.8540754, 0);
        public static final ClawState TOP_CUBE_CLAW_STATE = new ClawState(0.654, (Math.PI / 6));
        public static final ClawState MID_CUBE_CLAW_STATE = new ClawState(.6540754, 0);
        public static final ClawState LOW_CLAW_STATE = new ClawState(0.9, -0.925);
        public static final ClawState SUBSTATION_CLAW_STATE = new ClawState(.8540754 - 0.07, 0);
        public static final ClawState CUBE_INTAKE_CLAW_STATE = new ClawState(0.621, -0.925, false);
        public static final ClawState CONE_INTAKE_CLAW_STATE = new ClawState(0.521, -0.925, false);
    }

    public static final class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 21;
        public static final int RIGHT_MOTOR_ID = 19;
        public static final int LEFT_LIMIT_ID = 0;
        public static final int RIGHT_LIMIT_ID = 1;
        public static final int THROUGHBORE_ID_A = 3;
        public static final int THROUGHBORE_ID_B = 4;
        public static final double GEARBOX_GEARING = (9.0 / 62.0);
        public static final double SPROCKET_PD = Units.inchesToMeters(1.751);
        public static final double POSITION_CONVERSION_FACTOR_ROT_TO_METERS = Math.PI * SPROCKET_PD * 2.0;
        public static final double VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS = POSITION_CONVERSION_FACTOR_ROT_TO_METERS / 60.0;
        public static final double UPPER_LIMIT_METERS = Units.inchesToMeters(19.0 * 2.0);
        public static final double kS = 0.5;
        public static final double kG = 0.6;
        public static final double kV = 2.93;
        public static final double kA = 1.168;
        public static final double kP = 7.0; //7
        public static final double kI = 0.1;
        public static final double kD = 0.1;
        public static final double MAX_SPEED_M_S = MiscConstants.MAX_NEO_SPEED_RPM * VELOCITY_CONVERSION_FACTOR_RPM_TO_MPS / 2;
        public static final double MAX_ACCEL_M_S_S = MAX_SPEED_M_S / 6;
        public static final double GOAL_TOLERANCE_METERS = .03;
        public static final int CURRENT_LIMIT_AMPS = 40;

    }
    
    public static final class ArmConstants {
        public static final int TOP_ID = 26;
        public static final int BTM_ID = 27;
        public static final int SOL_ID = 2;
        public static final int THROUGHBORE_ID = 5; //on DIO ports
        public static final double POSITION_OFFSET_RAD = 2.5871;
        public static final int CURRENT_LIMIT_AMPS = 27;
        public static final double kS = 2.0;
        public static final double kG = 2.0;
        public static final double kV = 2.0;
        public static final double kA = 2.0;
        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double ARM_CONTROLLER_TOLERANCE_RAD = Units.degreesToRadians(3);
        public static final double ARM_CONTROLLER_TOLERANCE_RAD_JANK = Units.degreesToRadians(6);
        public static final double GEARBOX_GEARING = (1.0 / 100.0) * (17.0 / 20.0) * (20.0 / 32.0);
        public static final double MAX_SPEED_RAD_S = Units.rotationsPerMinuteToRadiansPerSecond(MiscConstants.MAX_NEO_SPEED_RPM * GEARBOX_GEARING);
        public static final double MAX_ACCEL_RAD_S_S = MAX_SPEED_RAD_S * 3;
        public static final double MAX_ACCEL_RAD_S_S_EXTENDED = MAX_SPEED_RAD_S;
        public static final double UPPER_LIMIT_RAD = 1.45;
        public static final double LOWER_LIMIT_RAD = -1;

        public static final double MANUAL_ARM_SPEED = .2;
    }

    public static final class RampPusherConstants {
        public static final int LEFT_ID = 13;
        public static final int RIGHT_ID = 16;
        public static final int THROUGHBORE_ID = 2;
        public static final int CURRENT_LIMIT_AMPS = 40;

        public static final double GEARING = (1.0 / 100.0);
        public static final double ENCODER_OFFSET_RAD = 4.22;
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double MAX_SPEED_RAD_S = Units.rotationsPerMinuteToRadiansPerSecond(MiscConstants.MAX_550_SPEED_RPM * GEARING) / 2;
        public static final double MAX_ACCEL_RAD_S_S = MAX_SPEED_RAD_S / 4;

        public static final double DEPLOYED_POS_RAD = 0; //HAS TO BE 0
        public static final double UNDEPLOYED_POS_RAD = 5.85 - ENCODER_OFFSET_RAD;
        public static final double CONTROLLER_TOLERANCE_RAD = Units.degreesToRadians(5);
        
    }

    public static final class ClawConstants {
        public static final int PROX_ID = 1; 
        public static final int LEFT_ID = 24;
        public static final int RIGHT_ID = 25;
        public static final int CURRENT_LIMIT_AMPS = 20;
        public static final int SOL_ID = 1;

        public static enum GAME_PIECE_STATE {
            CONE,
            CUBE,
            NO_GP
        }

        public static final double INTAKE_SPEED_DECIMAL = 0.3;
        public static final double CUBE_INTAKE_EXCESS_TIME_S = 0.15;
        public static final double OUTTAKE_SPEED_DECIMAL = -.3;
        public static final double OUTTAKE_SPEED_DECIMAL_SHOOTING = -1.0;
        public static final double CUBE_OUTTAKE_EXCESS_TIME_S = 0.4;

        public static final double PROX_VALUE = .875;
        public static final double PURPLE_THRESH = 600.0;
    }

    public static final class MiscConstants {

        public static final boolean FIELD_RELATIVE_SWITCHABLE = true;
        public static final boolean FIELD_RELATIVE_ON_START = false;
        public static final double MAX_NEO_SPEED_RPM = 5676.0;
        public static final double MAX_550_SPEED_RPM = 11000;
        public static final Pose2d DEBUG_RESET_POSE = new Pose2d(0, 0, new Rotation2d(0)); //have to flip before using if on red side
        public static final int MAX_SMAX_PERIODIC_FRAME_MS = 65535;
        public static final int THORUGHBORE_CONNECTION_HZ = 975;
    }

    public static final class LedConstants {

        public static final int[] PURPLE_CODE = {255, 0, 255};
        public static final int[] YELLOW_CODE = {255, 150, 0};
        public static final int[] GREEN_CODE = {0, 155, 100};
        public static final int[] BLUE_CODE = {0, 0, 255};
        public static final int[] RED_CODE = {255, 0, 0};
        public static final int[] WHITE_CODE = {100, 100, 100};
        public static final int[] CLEAR_CODE = {0, 0, 0};
    }
}

