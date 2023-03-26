package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.PartyMode;
import frc.robot.util.Logger;

public class Led extends SubsystemBase{

    public static final int PURPLE = 0;
    public static final int YELLOW = 1;
    public static final int BLUE = 5;
    public static final int GREEN = 2;
    public static final int RED = 3;
    public static final int WHITE = 4;
    public static final int CYAN = 6;
    public static final int ORANGE = 7;


    public static double ledCount = 0;
    public static final double LED_SPEED = .5;

    public static final int INTAKE = 0;
    public static final int INTAKE_FLOOR = 2;
    public static final int SCORING = 1;

    private boolean isPurple;

    private int state;

    public int[] rgb = new int[3];

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private boolean error;
    private boolean armError;

    public Led(){
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(120);
        error = false;
        armError = false;
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

    public void setError(boolean error){
        this.error = error;
    }
    
    public void setArmError(boolean error){
        this.armError = error;
    }

    public boolean isPurple(){
        return isPurple;
    }

    public void fill(int[] color) {
        // for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
        //     ledBuffer.setRGB(i, color[0], color[1], color[2]);
        //     ledBuffer.setRGB(i + 60, color[0], color[1], color[2]);
        // }
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
    }

    public void setLed(int i, int[] color) {
        if(i < 120 && i >=0){
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    @Override
    public void periodic() {  
        if(!PartyMode.running){    
            if (state == INTAKE && DriverStation.isEnabled()) 
            {
                if(isPurple)
                    setColor(PURPLE);
                else
                    setColor(YELLOW);
            } else if (state == SCORING &&DriverStation.isEnabled()) {
                setColor(CYAN);
            } else if (!DriverStation.isEnabled()) {
                setColor(RED);
            } else if(state = INTAKE_FLOOR && DriverStation.isEnabled()){
                setColor(ORANGE);
            }

            if(error){
                setColor(RED);
            }


            if(ledCount > 120){
                ledCount = 0;
            }

            // Cooler LED
            // ledCount += LED_SPEED;
            // for(int i = 0; i < 20; i++){
            //     int ledNum = (60 - Math.abs((int)(i + ledCount) % 60));
            //     setLed(ledNum, rgb);
            //     setLed(119 - ledNum, rgb);
            // }

            // if((int)(ledCount - LED_SPEED) != (int) ledCount){
            //     if(error){
            //         setLed((60 - Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.RED_CODE);
            //         setLed(119 - (60 - Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.RED_CODE);
            //     } else{
            //         setLed((60 - Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.WHITE_CODE);
            //         setLed(119 - (60 - Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.WHITE_CODE);
            //     }
            // }

            // cooler led
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
                    if(error){
                        setLed((60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.RED_CODE);
                        setLed(119 - (60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.RED_CODE);
                    } else{

                        if(RobotContainer.getLimelightLeft().targetVisible() || RobotContainer.getLimelightRight().targetVisible()){
                            setLed((60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.GREEN_CODE);
                            setLed(119 - (60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.GREEN_CODE);
                        } else{
                            setLed((60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.WHITE_CODE);
                            setLed(119 - (60 - Math.abs((int)(ledCount + (j * 20) - LED_SPEED) % 60)), LedConstants.WHITE_CODE);
                        }
                    }
                }
            }


            
        }
        Logger.post("isPurple", isPurple);

        

        led.setData(ledBuffer);
        led.start();
    }
}
