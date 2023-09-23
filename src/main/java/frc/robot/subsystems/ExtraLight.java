package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OdometryMath2023;

public class ExtraLight extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;
    private double ledMode;
    private PIDController rotPid;

    public ExtraLight(NetworkTable table) {
        this.table_m = table;
        rotPid = new PIDController(0.15, 0.0, 0.000);
        rotPid.setSetpoint(0);
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
        if (OdometryMath2023.inCommunity() && !DriverStation.isDisabled()) {
            setLight(true);
        } else {
            setLight(false);
        }
    }

    public boolean targetVisible() {
        return tv;
    }

    public double getTX() {
        return table_m.getEntry("tx").getDouble(0);
    }

    public double getAutoAlignCalc() {
        if (targetVisible()) {
            return rotPid.calculate(getTX());
        } else {
            return 0;
        }
    }

    public boolean lightOn() {
        return ledMode == 0;
    }
    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
