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
    public final double LED_SPEED = .1;

    public int[][] colors;

    public PartyMode(){
        addRequirements(led);
        colors = new int[][]{
            {255, 255, 255},
            {255, 224, 23},
            {255, 33, 204},
            {76, 22, 255},
            {43, 19, 69}
        };
    }

    @Override
    public void initialize(){
        ledCount = 0;
        running = true;
    }

    @Override
    public void execute(){
        ledCount += LED_SPEED;

            for(int i = 0; i < 6; i++){
                for(int j = 0; j < 5; j++){
                    led.setLed((i * 30) + (j * 6), colors[((int)ledCount + j)%5]);
                    led.setLed((i * 30) + (j * 6) + 1, colors[((int)ledCount + j)%5]);
                    led.setLed((i * 30) + (j * 6) + 2, colors[((int)ledCount + j)%5]);
                    led.setLed((i * 30) + (j * 6) + 3, colors[((int)ledCount + j)%5]);
                    led.setLed((i * 30) + (j * 6) + 4, colors[((int)ledCount + j)%5]);
                    led.setLed((i * 30) + (j * 6) + 5, colors[((int)ledCount + j)%5]);

                }
            }
        
            if(ledCount > 4){
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
