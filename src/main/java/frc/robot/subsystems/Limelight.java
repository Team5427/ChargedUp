package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;

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
        if(tv){
            double[] botPose = table_m.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            Translation2d translation = new Translation2d(botPose[0], botPose[1]);
            Rotation2d rotation2d = new Rotation2d(Math.toRadians(botPose[5]));

            return new Pose2d(translation, rotation2d);
        }
        
        
        return null;
    }

    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
