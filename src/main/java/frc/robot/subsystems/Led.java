package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotContainer;

public class Led extends SubsystemBase{

    public static final int PURPLE = 0;
    public static final int YELLOW = 1;
    public static final int BLUE = 5;
    public static final int GREEN = 2;
    public static final int RED = 3;
    public static final int WHITE = 4;

    public static double ledCount = 0;
    public static final double LED_SPEED = 1;

    public static final int INTAKE = 0;
    public static final int SCORING = 1;

    private boolean isPurple;

    private int state;

    public int[] rgb = new int[3];

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public Led(){
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(120);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        setColor(WHITE);

        fill(rgb);
    }

    public void togglePurple(){
        isPurple = !isPurple;
    }

    public void setState(int state){
        this.state = state;
    }

    public void setPurple(boolean isPurple) {
        this.isPurple = isPurple;
    }

    public boolean isPurple(){
        return isPurple;
    }

    public void fill(int[] color) {
        for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
            ledBuffer.setRGB(i + 60, color[0], color[1], color[2]);
        }
        
    }

    public void setColor(int c) {
        if(c == PURPLE){
            rgb = LedConstants.PURPLE_CODE;
        }
        if(c == YELLOW){
            rgb = LedConstants.YELLOW_CODE;
        }
        if(c == GREEN){
            rgb = LedConstants.GREEN_CODE;
        }
        if(c == RED){
            rgb = LedConstants.RED_CODE;
        }
        if(c == WHITE){
            rgb = LedConstants.WHITE_CODE;
        }
        if(c == BLUE){
            rgb = LedConstants.BLUE_CODE;
        }
    }

    public void setLed(int i, int[] color) {
        ledBuffer.setRGB(i, color[0], color[1], color[2]);
        ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    @Override
    public void periodic() {        
        if (state == INTAKE && DriverStation.isEnabled()) 
        {
            if(isPurple)
                setColor(PURPLE);
            else
                setColor(YELLOW);
        } else if (state == SCORING &&DriverStation.isEnabled()) {
            setColor(WHITE);
        } else if (!DriverStation.isEnabled()) {
            setColor(RED);
        }

        fill(rgb);

        led += LED_SPEED

        led.setData(ledBuffer);
        led.start();
    }
}
