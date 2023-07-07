package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.PartyMode;

public class Led extends SubsystemBase{

    public static final int PURPLE = 0;
    public static final int YELLOW = 1;
    public static final int BLUE = 5;
    public static final int GREEN = 2;
    public static final int RED = 3;
    public static final int WHITE = 4;
    public static final int CYAN = 6;
    public static final int ORANGE = 7;
    public static final int PINK = 8;


    public static double ledCount = 0;
    public static final double LED_SPEED = .5;
    public static final double APRILTAG_LED_SPEED = 2;

    public static final int INTAKE = 0;
    public static final int INTAKE_FLOOR = 2;
    public static final int SCORING = 1;
    public static final int SCORING_FLOOR = 3;

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

    public int getState(){
        return state;
    }

    public void setPurple(boolean isPurple) {
        this.isPurple = isPurple;
    }
    
    public boolean isPurple(){
        return isPurple;
    }

    public void fill(int[] color) {
        fillRange(0, 120, color);        
    }

    public void fillRange(int first, int last, int[] color){
        for (int i = first; i < ledBuffer.getLength() && i < last; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
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
        if(c == CYAN){
            rgb = new int[] {0, 200,100};
        }
        if(c == ORANGE){
            rgb = LedConstants.ORANGE_CODE;
        }
        if (c == PINK) {
            rgb = LedConstants.PINK_CODE;
        }
    }

    public void setLed(int i, int[] color) {
        if(i < 120 && i >=0){
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    @Override
    public void periodic() {  
        if (!PartyMode.running) {
            if (state == INTAKE && DriverStation.isEnabled()) 
            {
                if(isPurple)
                    setColor(PURPLE);
                else
                    setColor(YELLOW);
            } else if (state == SCORING && DriverStation.isEnabled()) {
                setColor(CYAN);
            } else if (state == SCORING_FLOOR && DriverStation.isEnabled()) {
                setColor(PINK);
            } else if(state == INTAKE_FLOOR && DriverStation.isEnabled()){
                setColor(ORANGE);
            } else if (!DriverStation.isEnabled()) {
                setColor(RED);
            }

            if(ledCount > 120){
                ledCount = 0;
            }

            ledCount += LED_SPEED;
            for(int j = 0; j < 3; j++){
                for(int i = 0; i < 8; i++){
                    int ledNum = (60 - Math.abs((int)(i + ledCount + (j * 20)) % 60));
                    setLed(ledNum, rgb);
                    setLed(119 - ledNum, rgb);
                }
            }

            for(int j = 0; j < 3; j++){
                if((int)(ledCount - LED_SPEED) != (int) ledCount){
                    if(RobotContainer.getLimelightLeft().targetVisible() || RobotContainer.getLimelightRight().targetVisible()){
                        setLed((60 - Math.abs((int)(ledCount + (j * 20) - APRILTAG_LED_SPEED) % 60)), LedConstants.GREEN_CODE);
                        setLed(119 - (60 - Math.abs((int)(ledCount + (j * 20) - APRILTAG_LED_SPEED) % 60)), LedConstants.GREEN_CODE);
                    } else{
                        setLed((60 - Math.abs((int)(ledCount + (j * 20) - APRILTAG_LED_SPEED) % 60)), LedConstants.WHITE_CODE);
                        setLed(119 - (60 - Math.abs((int)(ledCount + (j * 20) - APRILTAG_LED_SPEED) % 60)), LedConstants.WHITE_CODE);
                    }
                }
            }
        }

        led.setData(ledBuffer);
        led.start();
    }
}