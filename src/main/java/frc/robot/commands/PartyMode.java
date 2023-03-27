package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.RobotContainer;
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
            {255, 0, 52},
            {255, 52, 3},
            {255, 235, 0},
            {0, 220, 110},
            {0, 152, 195}
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

        // if(ledCount >= 1){
            for(int i = 0; i < 6; i++){
                for(int j = 0; j < 5; j++){
                    // int color = (int)(Math.random() * 5);
                    led.setLed((i * 30) + (j * 6), colors[(int)(ledCount + j) % 5]);
                    led.setLed((i * 30) + (j * 6) + 1, colors[(int)(ledCount + j) % 5]);
                    led.setLed((i * 30) + (j * 6) + 2, colors[(int)(ledCount + j) % 5]);
                    led.setLed((i * 30) + (j * 6) + 3, colors[(int)(ledCount + j) % 5]);
                    led.setLed((i * 30) + (j * 6) + 4, colors[(int)(ledCount + j) % 5]);
                    led.setLed((i * 30) + (j * 6) + 5, colors[(int)(ledCount + j) % 5]);

                }
            }
            // ledCount = 0;
        // }

        if(ledCount > 5){
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
