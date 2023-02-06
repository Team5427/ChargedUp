package frc.robot.commands.Auton;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.pathUtil.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup testAuton;
    private static Command swervePath1;
    public static void initAutons() {
        swervePath1 = SwervePathMaker.getCommand("StraightLinePath");

        testAuton = new SequentialCommandGroup(
            swervePath1
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });
    }
}
