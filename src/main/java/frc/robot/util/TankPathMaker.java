package frc.robot.util;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.RamseteController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TankDrive;

public class TankPathMaker {
    private static HashMap<String, PratsRamseteCommand> commandList = new HashMap<String, PratsRamseteCommand>();
    private static TankDrive driveTrain;

    public static void initPaths(String... sArgs) {
        Set<String> args = Set.of(sArgs);
        driveTrain = RobotContainer.getTank();
        args.forEach((name) -> {
            commandList.put(name, new PratsRamseteCommand(
                PathPlanner.loadPath(name, Constants.TankConstants.MAX_VELOCITY_AUTON_MPS, Constants.TankConstants.MAX_ACCEL_AUTON_MPSPS), 
                driveTrain::getPose, 
                new RamseteController(), 
                driveTrain.getFF(),
                Constants.TankConstants.TANK_KINEMATICS,
                driveTrain::getWheelSpeeds,
                driveTrain.getLeftPID(),
                driveTrain.getRightPID(),
                driveTrain::setVoltage,
                driveTrain::stopMotors,
                driveTrain
            ));
        });
    }

    public static PratsRamseteCommand getCommand(String name) {
        return commandList.get(name);
    }

    public static void resetPaths() {
        Set<String> s = commandList.keySet();
        commandList.clear();
        initPaths(s.toArray(String[]::new));
    }
}
