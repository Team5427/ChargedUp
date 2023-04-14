package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.util.OdometryMath2023;

public class Limelight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private int tag;
    private MedianFilter filterx;
    private LinearFilter linearFilterX;
    private MedianFilter filtery;
    private LinearFilter linearFilterY;
    private Pose2d lastPose2d;

    private double ledMode;

    public Limelight(NetworkTable table) {
        this.table_m = table;
        filterx = new MedianFilter(25);
        linearFilterX = LinearFilter.singlePoleIIR(0.25, 0.02);
        filtery = new MedianFilter(25);
        linearFilterY = LinearFilter.singlePoleIIR(0.25, 0.02);
        lastPose2d = new Pose2d();
    }

    @Override
    public void periodic() {
    }


    public boolean targetVisible() {
        tag = (int)table_m.getEntry("tid").getDouble(-1);
        tv = table_m.getEntry("tv").getDouble(0) == 1;

        return tv && tag != 4 && tag != 5;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }

    public Pose2d getEstimatedGlobalPose(){
        Pose2d ret;
        if(usingTarget()){
            double[] botPose = table_m.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            ret = filterPose(new Pose2d(botPose[0], botPose[1], RobotContainer.getSwerve().getRotation2d()));
            lastPose2d = ret;
        } else {
            filterx.reset();
            filtery.reset();
            linearFilterX.reset();
            linearFilterY.reset();
            ret = null;
        }
        return ret;
    }

    //applies IIR pole filter, and basic kalman filter
    public Pose2d filterPose(Pose2d pose) {
        if (!OdometryMath2023.inCommunity(pose) && MoveBotTo.reseed) {
            return lastPose2d;
        } else if (!OdometryMath2023.inField(pose)) {
            return lastPose2d;
        } else {
            double x = linearFilterX.calculate(filterx.calculate(pose.getX()));
            double y = linearFilterY.calculate(filtery.calculate(pose.getY()));
            return new Pose2d(x, y, pose.getRotation());
        }
    }

    public boolean usingTarget() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;

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
