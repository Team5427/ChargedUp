package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.SwervePathMaker;
import frc.robot.util.TankPathMaker;
import frc.robot.util.PratsRamseteCommand;
import frc.robot.util.PratsSwerveControllerCommand;

public class AutonSheet {
    public static SequentialCommandGroup testAuton;
    private static PratsSwerveControllerCommand swervePath1;
    private static PratsRamseteCommand tankPath1;
    public static void initAutons() {
        swervePath1 = SwervePathMaker.getCommand("swervePath1");
        tankPath1 = TankPathMaker.getCommand("tankPath1");

        testAuton = new SequentialCommandGroup(
            resetToFirstTrajectory(swervePath1),
            swervePath1,
            tankPath1
        ).andThen(() -> {
            SwervePathMaker.resetPaths();
            TankPathMaker.resetPaths();
        });
    }

    private static InstantCommand resetToFirstTrajectory(PratsSwerveControllerCommand firstDriveTrainCommand) {
        return new InstantCommand(() -> {
            RobotContainer.getSwerve().resetOdometry(firstDriveTrainCommand.getTrajectory().getInitialHolonomicPose());
        });
    }

    private static InstantCommand resetToFirstTrajectory(PratsRamseteCommand firstDriveTrainCommand) {
        return new InstantCommand(() -> {
            RobotContainer.getTank().resetOdometry(firstDriveTrainCommand.getTrajectory().getInitialPose());
        });
    }
}
