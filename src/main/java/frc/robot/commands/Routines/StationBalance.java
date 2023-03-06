package frc.robot.commands.Routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;

public class StationBalance extends SequentialCommandGroup {
    public StationBalance() {
        addCommands(
            new TurnAndTranslate(new Rotation2d(Math.PI), new Rotation2d(Math.PI), 2.5, 1.25),
            new Balance()
        );
    }
}
