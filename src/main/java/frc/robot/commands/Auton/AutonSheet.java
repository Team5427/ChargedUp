package frc.robot.commands.Auton;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.PratsSwerveControllerCommand;
import frc.robot.util.SwervePathMaker;

public class AutonSheet {
    public static SequentialCommandGroup testAuton;
    private static PratsSwerveControllerCommand swervePath1;
    public static void initAutons() {
        swervePath1 = SwervePathMaker.getCommand("StraightLinePath");

        testAuton = new SequentialCommandGroup(
            resetToFirstTrajectory(swervePath1),
            swervePath1
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
        });
    }

    private static InstantCommand resetToFirstTrajectory(PratsSwerveControllerCommand firstDriveTrainCommand) {
        return new InstantCommand(() -> {
            RobotContainer.getSwerve().resetOdometry(firstDriveTrainCommand.getTrajectory().getInitialHolonomicPose());
        });
    }
}
