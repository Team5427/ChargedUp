package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private double redOffset, blueOffset;

    private double ledMode;

    public Limelight(NetworkTable table, double redOffsetRad, double blueOffsetRad) {
        this.table_m = table;
        filterx = new MedianFilter(5);
        linearFilterX = LinearFilter.singlePoleIIR(0.1, 0.02);
        filtery = new MedianFilter(5);
        linearFilterY = LinearFilter.singlePoleIIR(0.1, 0.02);
        this.redOffset = redOffsetRad;
        this.blueOffset = blueOffsetRad;
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
        } else {
            double x = RobotContainer.getSwerve().getPose().getX();
            double y = RobotContainer.getSwerve().getPose().getY();
            filterx.calculate(x);
            filtery.calculate(y);
            linearFilterX.calculate(x);
            linearFilterY.calculate(y);
            ret = null;
        }
        return ret;
    }

    public Rotation2d getTagBasedRotation() {
        if (usingTarget()) {
            double[] botPose = table_m.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            double rot = botPose[5];
            // System.out.println(Math.toRadians(rot));
            if (OdometryMath2023.isBlue()) {
                return new Rotation2d(Math.toRadians(rot) - blueOffset);    
            } else {
                return new Rotation2d(Math.toRadians(rot) - redOffset);
            }
        } else {
            return null;
        }
    }

    //applies IIR pole filter, and basic kalman filter
    public Pose2d filterPose(Pose2d pose) {
        if (!OdometryMath2023.inCommunity(pose) && MoveBotTo.running) {
            return null;
        } else if (!OdometryMath2023.inField(pose)) {
            return null;
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
