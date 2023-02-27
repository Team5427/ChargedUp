package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Routines.TurnAndTranslate;
import frc.robot.util.OdometryMath2023;

public class SubRoutineSheet {
    public static ParallelRaceGroup substationIntake;

    public static void initSubRoutines() {
        if (OdometryMath2023.isBlue()) {
            substationIntake = new ParallelRaceGroup(
                new TurnAndTranslate(0.0, 0.0, .5, 1)
                
            );
        } else {
            substationIntake = new ParallelRaceGroup(new TurnAndTranslate(Math.PI, Math.PI, .5, 1));
        }
    }
}
