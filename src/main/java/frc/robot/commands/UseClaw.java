package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;

public class UseClaw extends CommandBase {

    Claw claw;

    public UseClaw() {
        // claw = RobotContainer.getClaw()
        addRequirements(claw);
    }
}
