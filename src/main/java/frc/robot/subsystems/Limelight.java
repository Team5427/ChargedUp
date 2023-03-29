package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private int tag;
    private MedianFilter filterx;
    private MedianFilter filtery;

    private double ledMode;

    public Limelight(NetworkTable table) {
        this.table_m = table;
        filterx = new MedianFilter(20);
        filtery = new MedianFilter(20);
    }

    @Override
    public void periodic() {
        tag = (int)table_m.getEntry("tid").getDouble(-1);
        tv = table_m.getEntry("tv").getDouble(0) == 1;
    }


    public boolean targetVisible() {
        return tv && tag != 4 && tag != 5;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }

    public Pose2d getEstimatedGlobalPose(){
        if(usingTarget()){
            double[] botPose = table_m.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            Translation2d translation = new Translation2d(
                filterx.calculate(botPose[0]),
                filtery.calculate(botPose[1])
            );

            return new Pose2d(translation, RobotContainer.getSwerve().getRotation2d());
        } else {
            filterx.reset();
            filtery.reset();
            return null;
        }
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
