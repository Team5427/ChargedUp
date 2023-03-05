package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstants;
import frc.robot.util.Logger;

public class Led extends SubsystemBase{

    public static final int BLINK = 0;
    public static final int SOLID = 1;
    public static final int WATERFALL = 2;
    public static final int LOADING = 3;

    public static final int PURPLE = 0;
    public static final int YELLOW = 1;
    public static final int GREEN = 2;
    public static final int RED = 3;
    public static final int WHITE = 4;

    public int[] rgb = new int[3];

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int state;
    private int color;

    private int blinkCount;

    public boolean isPurple;
    private Timer timer;
    private int position;

    public Led(){
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(120);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

        state = SOLID;
        setColor(WHITE);

        fill(rgb);

        timer = new Timer();
        timer.reset();
        timer.start();

        position = 1;

        blinkCount = 0;
        isPurple = false;
    }

    public void togglePurple(){
        isPurple = !isPurple;
    }

    public boolean isPurple(){
        return isPurple;
    }

    public void fill(int[] color) {
        for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
            ledBuffer.setRGB(i + 60, color[0], color[1], color[2]);
        }
        led.setData(ledBuffer);
        led.start();
    }

    public void setpoint(int i, int[] color) {
        ledBuffer.setRGB(i, color[0], color[1], color[2]);
        ledBuffer.setRGB(i, color[0], color[1], color[2]);

        led.setData(ledBuffer);
        led.start();
    }

    public void stop() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setState(int s) {
        state = s;
    }

    public void setColor(int c) {
        color = c;
        if(color == PURPLE){
            setRGB(LedConstants.PURPLE);
        }
        if(color == YELLOW){
            setRGB(LedConstants.YELLOW);
        }
        if(color == GREEN){
            setRGB(LedConstants.GREEN);
        }
        if(color == RED){
            setRGB(LedConstants.RED);
        }
        if(color == WHITE){
            setRGB(LedConstants.WHITE);
        }
    }

    public void setRGB(int[] color) {
        rgb = color;
    }

    @Override
    public void periodic() {
        Logger.post("led purple", isPurple);


        
        if (state == BLINK) {
            
            blinkCount = 0;

            fill(rgb);
        }
        else if (state == WATERFALL) {
            blinkCount = 0;

            fill(rgb);
            
        }
        else if (state == SOLID){
            blinkCount = 0;

            fill(rgb);
        }
        else if (state == LOADING){
            
            // rgb = LedConstants.WHITE;

            // if(blinkCount < 3){
            //     if (timer.get() > .5) {
            //         fill(rgb);
            //         timer.reset();
            //         timer.start();
            //     }
            //     else if (timer.get() > .25) {
            //         fill(rgb);
            //     }

            //     blinkCount++;
            // } else{


                if(isPurple)
                    fill(LedConstants.PURPLE);
                else
                    fill(LedConstants.YELLOW);
            // }
        }
    }
}
