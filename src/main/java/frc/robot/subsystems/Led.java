package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
