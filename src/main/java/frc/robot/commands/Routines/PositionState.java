package frc.robot.commands.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.*;
import frc.robot.util.OdometryMath2023;

public class PositionState {

    public static Pose2d getScoringPose(RoutineConstants.SCORING_TYPE type) {
        Pose2d ret;
        switch(type) {
            case BOTTOM_CONE: {
                ret = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT;
                break;
            } case CUBE: {
                ret = RoutineConstants.CUBE_SCORING_POSE_DEFAULT;
                break;
            } case TOP_CONE: {
                ret = RoutineConstants.TOP_CONE_SCORING_POSE_DEFAULT;
                break;
            } default: {
                ret = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT;
            }
        }
        ret = new Pose2d(
            ret.getX(), 
            (ret.getY() + OdometryMath2023.getScoringLevel() * RoutineConstants.SCORING_LEVEL_OFFSET_METERS), 
            ret.getRotation()
        );
        if (!OdometryMath2023.isBlue()) {
            ret = OdometryMath2023.flip(ret);
        }
        return ret;
    }

    public static Pose2d getSubstationPose(RoutineConstants.SUBSTATION_TYPE type) {
        Pose2d ret;
        switch(type) {
            case LEFT_SS: {
                if (OdometryMath2023.isBlue()) {
                    ret = RoutineConstants.TOP_SUBSTATION_POSE_DEFAULT;
                } else {
                    ret = RoutineConstants.BOTTOM_SUBSTATION_POSE_DEFAULT;
                }
            } case RIGHT_SS: {
                if (!OdometryMath2023.isBlue()) {
                    ret = RoutineConstants.TOP_SUBSTATION_POSE_DEFAULT;
                } else {
                    ret = RoutineConstants.BOTTOM_SUBSTATION_POSE_DEFAULT;
                }
            } default: {
                ret = RoutineConstants.BOTTOM_SUBSTATION_POSE_DEFAULT;
            }
        }
        if (!OdometryMath2023.isBlue()) {
            ret = OdometryMath2023.flip(ret);
        }
        return ret;
    }
}
