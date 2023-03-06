package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Logger;

public class Limelight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private Pose2d lastPose = null;
    private Pose2d currentPose = null;
    private boolean flickered = false;

    private double ledMode;

    public Limelight(NetworkTable table) {
        this.table_m = table;
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
    }

    public boolean targetVisible() {
        return tv;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }

    public Pose2d getEstimatedGlobalPose(){
        if(usingTarget()){
            double[] botPose = table_m.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            Translation2d translation = new Translation2d(botPose[0], botPose[1]);

            return new Pose2d(translation, RobotContainer.getSwerve().getRotation2d());
        } else {
            return null;
        }
    }

    public Pose2d getAdjustedGlobalPose() {
        if (!flickered) {
            lastPose = currentPose;
        }
        currentPose = getEstimatedGlobalPose();
        if (currentPose == null) {
            flickered = false;
            return currentPose;
        } else {
            if (lastPose == null) {
                flickered = false;
                return currentPose;
            } else {
                if (!flicker(lastPose, currentPose)) {
                    flickered = false;
                    return currentPose;
                } else {
                    flickered = true;
                    return lastPose;
                }
            }
        }
    }

    public boolean flicker(Pose2d last, Pose2d current) {
        Logger.post("X Flicker Val", Math.abs(current.getX() - last.getX()));
        Logger.post("Y Flicker Val", Math.abs(current.getY() - last.getY()));
        Logger.post("Theta Flicker Val", (Math.abs(current.getRotation().minus(last.getRotation()).getRadians())));

        // return (
        //     (Math.abs(current.getX() - last.getX()) > JoystickConstants.REGULAR_SPEED_M_S * 0.02) ||
        //     (Math.abs(current.getY() - last.getY()) > JoystickConstants.REGULAR_SPEED_M_S * 0.02) ||
        //     (Math.abs(current.getRotation().minus(last.getRotation()).getRadians())) > JoystickConstants.REGULAR_ANGULAR_SPEED_RAD_S * 0.02
        // );

        return false;
    }

    public boolean usingTarget() {
        return tv;
    }

    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
