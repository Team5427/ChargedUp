package frc.robot.commands.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;
import frc.robot.util.OdometryMath2023;

public class StationBalance extends SequentialCommandGroup {
    public StationBalance() {
        double rot = 0;
        if (!OdometryMath2023.isBlue()) {
            rot = Math.PI;
        }
        addCommands(
            new TurnAndTranslate(rot, rot, 2, 1.25),
            new Balance()
        );
    }
}
