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
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.RobotContainer;

public class OdometryMath2023 extends SubsystemBase {

    private static Pose2d robotPose;
    private static final double fieldWidth = Units.feetToMeters(54);
    private static final double fieldHeight = Units.feetToMeters(27);
    private static final double scoringY1 = RoutineConstants.Y_LEVEL_1_METERS;
    private static final double scoringY2 = RoutineConstants.Y_LEVEL_2_METERS;
    private static final double scoringY3 = RoutineConstants.Y_LEVEL_3_METERS;
    private static Pose2d limelightLeftPose;
    private static Pose2d limelightRightPose;
    private static Rotation2d limelightLeftTagRot;
    private static Rotation2d limelightRightTagRot;

    private void log() {
        if (tagRotation() != null) {
            Logger.post("move bot to thing", OdometryMath2023.tagRotation().getRadians());
            // Logger.post("left move bot to thing", limelightLeftTagRot.getRadians());
            // Logger.post("right move bot to thing", limelightRightTagRot.getRadians());
        }
    }

    @Override
    public void periodic() {
        limelightLeftPose = RobotContainer.getLimelightLeft().getEstimatedGlobalPose();
        limelightRightPose = RobotContainer.getLimelightRight().getEstimatedGlobalPose();
        limelightLeftTagRot = RobotContainer.getLimelightLeft().getTagBasedRotation();
        limelightRightTagRot = RobotContainer.getLimelightRight().getTagBasedRotation();
        robotPose = RobotContainer.getSwerve().getPose();

        if (!DriverStation.isAutonomous()) {
            reseedOdometry();
        }
        log();
    }

    public static void reseedOdometry() {
        if (limelightLeftPose != null && limelightRightPose != null) {
            RobotContainer.getSwerve().updateVision(averagePose(limelightLeftPose, limelightRightPose));
        } else if ((limelightLeftPose == null && limelightRightPose != null)) {
            RobotContainer.getSwerve().updateVision(limelightRightPose);
        } else if (limelightLeftPose != null && limelightRightPose == null) {
            RobotContainer.getSwerve().updateVision(limelightLeftPose);
        } else {
            return;
        }
    }

    public static Rotation2d tagRotation() {
        if (limelightLeftTagRot != null && limelightRightTagRot != null) {
            // return new Rotation2d(
            //     (((limelightLeftTagRot.getRadians() < 0)?(limelightLeftTagRot.getRadians() + (2 * Math.PI)):limelightLeftTagRot.getRadians()) + 
            //     ((limelightRightTagRot.getRadians() < 0)?(limelightRightTagRot.getRadians() + (2 * Math.PI)):limelightRightTagRot.getRadians())/2)
            //     );
            Rotation2d num = limelightLeftTagRot.rotateBy(limelightRightTagRot).div(2);
            if (Math.abs(robotPose.getRotation().minus(num).getRadians()) > (Math.PI/2)) {
                return num.rotateBy(new Rotation2d(Math.PI));
            } else {
                return num;
            }
        } else if ((limelightLeftTagRot == null && limelightRightTagRot != null)) {
            return limelightRightTagRot;
        } else if (limelightLeftTagRot != null && limelightRightTagRot == null) {
            return limelightLeftTagRot;
        } else {
            return null;
        }
    }

    public static double smartArcAngle(double inputX, double inputY, double distance) {
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

    public static Pose2d averagePose(Pose2d A, Pose2d B) {
        return new Pose2d(
            (A.getX() + B.getX())/2,
            (A.getY() + B.getY())/2,
            A.getRotation().rotateBy(B.getRotation()).div(2)    
        );
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

    public static void doPeriodicFrame(CANSparkMax... motor) {
        for (CANSparkMax m : motor) {
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus3, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus4, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus5, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus6, MiscConstants.MAX_SMAX_PERIODIC_FRAME_MS);
        }
    }

    public static boolean inScoringSpot() {
        if (isBlue()) {
            return robotPose.getX() < RoutineConstants.CLOSE_COMMUNITY_X_METERS;
        } else {
            return robotPose.getX() > (fieldWidth - RoutineConstants.CLOSE_COMMUNITY_X_METERS);
        }
    }

    public static boolean onScoringSide() {
        if (isBlue()) {
            return robotPose.getX() < fieldWidth/2;
        } else {
            return robotPose.getX() > fieldWidth/2;
        }
    }

    public static boolean inCommunity(Pose2d pose) {
        if (isBlue()) {
            return pose.getX() < RoutineConstants.IN_COMMUNITY_X && pose.getY() < RoutineConstants.IN_COMMUNITY_Y;
        } else {
            return pose.getX() > (fieldWidth - RoutineConstants.IN_COMMUNITY_X) && pose.getY() < RoutineConstants.IN_COMMUNITY_Y;
        }
    }

    public static boolean inCommunity() {
        if (isBlue()) {
            return robotPose.getX() < RoutineConstants.IN_COMMUNITY_X && robotPose.getY() < RoutineConstants.IN_COMMUNITY_Y;
        } else {
            return robotPose.getX() > (fieldWidth - RoutineConstants.IN_COMMUNITY_X) && robotPose.getY() < RoutineConstants.IN_COMMUNITY_Y;
        }
    }

    public static boolean facingForward(double toleranceDeg) {
        double toleranceRad = Math.toRadians(toleranceDeg);
        if (isBlue()) {
            return Math.abs(Math.abs(robotPose.getRotation().getRadians()) - Math.PI) < toleranceRad;
        } else {
            return Math.abs(robotPose.getRotation().getRadians()) < toleranceRad;
        }
    }

    public static boolean inField(Pose2d pose) {
        return (pose.getX() < fieldWidth && pose.getX() > 0) && (pose.getY() < fieldHeight && pose.getY() > 0);
    }
}
