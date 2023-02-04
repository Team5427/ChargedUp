package frc.robot.commands.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.*;
import frc.robot.util.OdometryMath2023;

public class PositionState {

    public static Pose2d getPositionPose(RoutineConstants.POSITION_TYPE type) {
        Pose2d ret;
        switch(type) {
            case LEFT_CONE: {
                if (OdometryMath2023.isBlue()) {
                    ret = RoutineConstants.TOP_CONE_SCORING_POSE_DEFAULT;
                } else {
                    ret = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT;
                }
                break;
            } case CUBE: {
                ret = RoutineConstants.CUBE_SCORING_POSE_DEFAULT;
                break;
            } case RIGHT_CONE: {
                if (!OdometryMath2023.isBlue()) {
                    ret = RoutineConstants.TOP_CONE_SCORING_POSE_DEFAULT;
                } else {
                    ret = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT;
                }                
                break;
            } default: {
                ret = RoutineConstants.BOTTOM_CONE_SCORING_POSE_DEFAULT;
            }
        }
        if (
            type.equals(RoutineConstants.POSITION_TYPE.LEFT_CONE) || 
            type.equals(RoutineConstants.POSITION_TYPE.RIGHT_CONE) || 
            type.equals(RoutineConstants.POSITION_TYPE.CUBE)
        ) {
            ret = new Pose2d(
                ret.getX(), 
                (ret.getY() + OdometryMath2023.getScoringLevel() * RoutineConstants.SCORING_LEVEL_OFFSET_METERS), 
                ret.getRotation()
            );
        }
        if (!OdometryMath2023.isBlue()) {
            ret = OdometryMath2023.flip(ret);
        }
        return ret;
    }
}
