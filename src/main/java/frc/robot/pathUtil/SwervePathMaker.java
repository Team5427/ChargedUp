package frc.robot.pathUtil;

import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class SwervePathMaker {
    private static HashMap<String, SequentialCommandGroup> commandList = new HashMap<String, SequentialCommandGroup>();
    private static SwerveDrive driveTrain;
    private static PIDController xTranslationPID, yTranslationPID;
    private static PIDController thetaPID;

    public static void initPaths(String... sArgs) {
        Set<String> args = Set.of(sArgs);
        xTranslationPID = new PIDController(Constants.SwerveConstants.AUTON_TRANSLATION_P, 0, 0);
        yTranslationPID = new PIDController(Constants.SwerveConstants.AUTON_TRANSLATION_P, 0, 0);
        thetaPID = new PIDController(Constants.SwerveConstants.AUTON_THETA_P, 0, 0);
        driveTrain = RobotContainer.getSwerve();
        args.forEach((name) -> {
            CustomTrajectory traj = CustomPlanner.loadPath(name, SwerveConstants.MAX_AUTON_SPEED_M_PER_S, SwerveConstants.MAX_AUTON_ACCEL_M_PER_S2);
            commandList.put(name, new SequentialCommandGroup(
                new InstantCommand(() -> {
                    driveTrain.resetOdometry(OdometryMath2023.isBlue() ? traj.getInitialHolonomicPose() : OdometryMath2023.flip(traj.getInitialHolonomicPose()));
                }, driveTrain),
                new CustomSwerveControllerCommand(
                    traj,
                    driveTrain::getPose, 
                    Constants.SwerveConstants.SWERVE_DRIVE_KINEMATICS, 
                    xTranslationPID,
                    yTranslationPID,
                    thetaPID,
                    driveTrain::setModules,
                    true,
                    driveTrain)
                )
            );
        });
    }

    public static SequentialCommandGroup getCommand(String name) {
        return commandList.get(name);
    }

    public static void resetPaths() {
        Set<String> s = commandList.keySet();
        commandList.clear();
        initPaths(s.toArray(String[]::new));
    }
}
