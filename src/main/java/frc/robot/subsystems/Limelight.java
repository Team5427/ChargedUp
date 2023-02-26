package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.OdometryMath2023;

public class Limelight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private double ta;
    private double tx;
    private double taThresh;

    private double ledMode;

    public Limelight(NetworkTable table) {
        this.table_m = table;
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
        ta = table_m.getEntry("ta").getDouble(0);
        tx = table_m.getEntry("tx").getDouble(0);
        
        if (OdometryMath2023.isBlue()) {
            if (RobotContainer.getSwerve().getPose().getX() > Units.feetToMeters(40)) {
                taThresh = 0.5;
            } else {
                taThresh = 0.8;
            }
        } else {
            if (RobotContainer.getSwerve().getPose().getX() < Units.feetToMeters(14)) {
                taThresh = 0.5;
            } else {
                taThresh = 0.8;
            }
        }
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
            Rotation2d rotation2d = new Rotation2d(Math.toRadians(botPose[5]));

            return new Pose2d(translation, rotation2d);
        } else {
            return null;
        }
    }

    public boolean usingTarget() {
        return tv && (ta > taThresh) && (Math.abs(tx) < 20);
    }

    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
