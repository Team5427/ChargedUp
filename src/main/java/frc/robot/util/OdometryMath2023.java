package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class OdometryMath2023 extends SubsystemBase {

    private static Pose2d robotPose;
    private static Rotation2d gyroYaw;

    private void log() {
        // Logger.post("easiest turn", OdometryMath2023.robotAngleToTarget(new Translation2d(0, 0)));
    }

    @Override
    public void periodic() {
        robotPose = RobotContainer.getSwerve().getPose();
        gyroYaw = robotPose.getRotation();
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
}
