package frc.robot.commands.Routines;

public class ClawState {

    private double height;
    private double angle;

    public ClawState(double h, double a) {
        this.height = h;
        this.angle = a;
    }

    public double getHeight() {
        return height;
    }

    public double getAngle() {
        return angle;
    }
}
