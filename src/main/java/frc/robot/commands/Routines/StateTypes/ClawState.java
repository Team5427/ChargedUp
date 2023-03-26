package frc.robot.commands.Routines.StateTypes;

public class ClawState {

    private double height;
    private double angle;
    private boolean extended;
    private double armDelay;
    private boolean retracted;

    public ClawState(double h, double a) {
        this.height = h;
        this.angle = a;
        this.extended = false;
        this.retracted = false;
    }

    public ClawState(double h, double a, boolean e) {
        this.height = h;
        this.angle = a;
        this.extended = e;
        this.retracted = false;
    }

    public ClawState(double h, double a, boolean e, boolean r) {
        this.height = h;
        this.angle = a;
        this.extended = e;
        this.retracted = r;
    }

    public double getHeight() {
        return height;
    }

    public double getAngle() {
        return angle;
    }

    public boolean getExtended() {
        return extended;
    }

    public double getArmDelay(){
        return armDelay;
    }

    public boolean getRetracted() {
        return retracted;
    }
}
