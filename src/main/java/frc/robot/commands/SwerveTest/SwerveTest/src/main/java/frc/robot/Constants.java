package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final int TURN_MOTOR_ID = 0;
    public static final int SPEED_MOTOR_ID = 0;

    public static final int CANCODER_ID = 0;

    public static final double TURN_GEARING = 7.0/105.0;
    public static final double DRIVE_GEARING = (14.0/50.0) * (28.0/16.0) * (15.0/45.0);

    public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);
    public static final double MAX_SPEED_PHYSICAL_MPS = (Units.rotationsPerMinuteToRadiansPerSecond(5676) * WHEEL_DIAMETER_M * DRIVE_GEARING)/(2.0);

    public static final double MAX_CONTROL_SPEED_MPS = MAX_SPEED_PHYSICAL_MPS * 0.75;
    public static final double MAX_CONTROL_ACCEL_MPSS = MAX_CONTROL_SPEED_MPS * 2;

    public static final double TURN_CONVERSION_FACTOR_ROT_TO_RAD = TURN_GEARING * 2 * Math.PI;
    public static final double SPEED_CONVERSION_FACTOR_RPM_TO_MPS = (Math.PI * WHEEL_DIAMETER_M * DRIVE_GEARING) / (60.0);

    public static final double TURN_KP = 0.0;
    public static final double TURN_KD = 0.0;
    
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_FF = 0.0;

}