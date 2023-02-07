package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MiscConstants;

public class OdometryMath2023 extends SubsystemBase {

    private static Pose2d robotPose;
    private static final double fieldWidth = Units.feetToMeters(54);
    private static final double fieldHeight = Units.feetToMeters(27);
    private static final double scoringY1 = Units.feetToMeters(0); //FIXME
    private static final double scoringY2 = Units.feetToMeters(0); //FIXME
    private static final double scoringY3 = Units.feetToMeters(0); //FIXME y of gaurdrail

    private void log() {
        Logger.post("easiest turn", OdometryMath2023.robotAngleToTarget(new Translation2d(0, 0)));
    }

    @Override
    public void periodic() {
        robotPose = RobotContainer.getSwerve().getPose();
        log();
    }

    private static double smartArcAngle(double inputX, double inputY, double distance) {
        if (inputX < 0 ) {
            return (Math.PI - Math.asin(inputY/distance));
        } else {
            return Math.asin(inputY/distance);
        }
    }

    public static double robotAngleToTarget(Translation2d targetTranslationMeters) {
        Translation2d trans = robotPose.getTranslation().minus(targetTranslationMeters);
        Rotation2d rot = new Rotation2d(smartArcAngle(trans.getX(), trans.getY(), Math.hypot(trans.getX(), trans.getY())));
        return rot.minus(robotPose.getRotation()).getRadians(); //test w swerve

    }

    public static int getScoringLevel() {
        //trans1 is bottom left, trans2 is top right of box
        double y = robotPose.getY();
        if (y <= scoringY1) {
            return 0;
        } else if (y <= scoringY2) {
            return 1;
        } else if (y <= scoringY3) {
            return 2;
        } else {
            return 3;
        }
    }

    public static Pose2d flip(Pose2d pose) {
        Translation2d trans = pose.getTranslation();
        Rotation2d rot = pose.getRotation();
        return new Pose2d(flip(trans), flip(rot));
    }

    public static Translation2d flip(Translation2d trans) {
        double x = trans.getX();
        x = (fieldWidth - x);
        return new Translation2d(x, trans.getY());
    }

    public static Rotation2d flip(Rotation2d rot) {
        double x = rot.getCos();
        x *= -1;
        return new Rotation2d(x, rot.getSin());
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().equals(Alliance.Blue);
    }

    public static void doPeriodicFrame(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
    }
}
