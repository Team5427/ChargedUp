package frc.robot.subsystems;

import java.time.Period;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Logger;

public class Led extends SubsystemBase{
    public boolean purple;
    public Led(){
        
    }

    public void togglePurple(){
        purple = !purple;
    }

    public boolean isPurple(){
        return purple;
    }

    @Override
    public void periodic() {
        Logger.post("led purple", purple);
    }
}
