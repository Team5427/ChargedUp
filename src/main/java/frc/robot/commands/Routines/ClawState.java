package frc.robot.commands.Routines;

public class ClawState {

    private double height;
    private double angle;
    private boolean extended;
    private double armDelay;

    public ClawState(double h, double a) {
        this.height = h;
        this.angle = a;
        this.extended = false;
        this.armDelay = 0;
    }

    public ClawState(double h, double a, boolean e, double armDelay){
        this.height = h;
        this.angle = a;
        this.extended = e;
        this.armDelay = armDelay;
    }

    public ClawState(double h, double a, boolean e) {
        this.height = h;
        this.angle = a;
        this.extended = e;
        this.armDelay = 0;
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
}
