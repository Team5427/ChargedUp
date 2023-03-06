package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TapeLight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private double ty;
    private double tx;

    private double ledMode;

    public TapeLight(NetworkTable table) {
        this.table_m = table;
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
        ty = table_m.getEntry("ty").getDouble(0);
        tx = table_m.getEntry("tx").getDouble(0);
    }

    public boolean targetVisible() {
        return tv;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }

    public boolean hasTarget() {
        return tv;
    }

    public double getVertical() {
        return ty;
    }

    public double getHorizontal() {
        return tx;
    }

    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
