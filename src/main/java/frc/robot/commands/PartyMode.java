package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.Led;

public class PartyMode extends CommandBase{
    Led led = RobotContainer.getLed();

    public static boolean running = false;

    public double ledCount = 0;
    public final double LED_SPEED = 1;

    public boolean explode = false;
    public double locationMultiplier = 0.4;
    public double locationAcc =  1.1;

    public int[][] colors;

    public PartyMode(){
        addRequirements(led);
        colors = new int[][]{
            {255, 119, 0},
            {255, 42, 0},
            {255, 130, 100},
            {255, 100, 0},
            {230, 50, 105},
            {255, 0, 100},
            {255, 155, 125},
            {180, 0, 255},
            {255, 255, 0},
            {155, 130, 0}
        };
    }

    @Override
    public void initialize(){
        ledCount = 0;
        explode = false;
        locationMultiplier = 0.4;

        running = true;
    }

    @Override
    public void execute(){
        if(!explode){

            ledCount += LED_SPEED;

            for(int i = 0; i < 5; i++){
                int ledNum = (Math.abs((int)(i + ledCount) % 60));
                led.setLed(ledNum, LedConstants.WHITE_CODE);
                led.setLed(119 - ledNum, LedConstants.WHITE_CODE);
            }

            if((int)(ledCount - LED_SPEED) != (int) ledCount){
                led.setLed((Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.CLEAR_CODE);
                led.setLed(119 - (Math.abs((int)(ledCount - LED_SPEED) % 60)), LedConstants.CLEAR_CODE);
                
            }

            if(ledCount > 60){
                ledCount = 0;

                explode = true;
            }
        } else{
            for(int i = 0; i <  colors.length; i++){
                if((int)(i * locationMultiplier) <= 60){
                    led.setLed(60 - ((int)(i * locationMultiplier)), LedConstants.CLEAR_CODE);
                    led.setLed(119 - (60 - ((int)(i * locationMultiplier))), LedConstants.CLEAR_CODE);
                }

                if((int)(i * locationMultiplier * locationAcc) <= 60){
                    led.setLed(60 - ((int)(i * locationMultiplier * locationAcc)), colors[i]);
                    led.setLed(119 - (60 - ((int)(i * locationMultiplier * locationAcc))), colors[i]);
                    led.setLed(60 - ((int)(i * locationMultiplier * locationAcc) + 1), colors[i]);
                    led.setLed(119 - (60 - ((int)(i * locationMultiplier * locationAcc) + 1)), colors[i]);
                }
            }

            locationMultiplier *= locationAcc;
            if(locationMultiplier > 15){
                explode = false;
                led.fill(LedConstants.CLEAR_CODE);
                locationMultiplier = .4;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.FLIP_COLOR);
    }

    @Override
    public void end(boolean interrupted){
        running = false;
    }
}
