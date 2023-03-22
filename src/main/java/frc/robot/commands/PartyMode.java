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
    public final double LED_SPEED = 5;

    public int[][] colors;

    public PartyMode(){
        addRequirements(led);
        colors = new int[][]{
            {255, 130, 130},
            {255, 180, 90},
            {255, 230, 100},
            {170, 255, 90},
            {90, 250, 205},
            {90, 190, 240},
            {155, 95, 225},
            {180, 0, 255},
            {225, 95, 240},
            {255, 130, 200}
        };
    }

    @Override
    public void initialize(){
        ledCount = 0;
        running = true;
    }

    @Override
    public void execute(){
        ledCount++;

        if(ledCount >= LED_SPEED){
            for(int i = 0; i < 60; i++){
                led.setLed(i, colors[(int)(Math.random() * 10)]);
                led.setLed(i + 1, colors[(int)(Math.random() * 10)]);
            }
            ledCount = 0;
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
