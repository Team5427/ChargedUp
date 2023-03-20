package frc.robot.commands.Routines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.Routines.StateTypes.ClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Logger;
import frc.robot.util.OdometryMath2023;

public class MoveClawTo extends CommandBase {
    
    private ClawState setPoint;
    private Timer timer;
    private Arm arm;
    private Elevator elevator;
    public static boolean goodToRelease;

    public MoveClawTo(ClawState setPoint) {
        this.setPoint = setPoint;
        arm = RobotContainer.getArm(); //commenting out since havent implemented in robotcontainer yet
        elevator = RobotContainer.getElevator();
        timer = new Timer();
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        goodToRelease = false;
        timer.reset();
        timer.start();
        Logger.post("Arm Finished", false);

    }

    @Override
    public void execute() {
        arm.setAngle(setPoint.getAngle());
        if ((RobotContainer.getSwerve().getPose().getX() > Units.feetToMeters(33) && OdometryMath2023.isBlue()) || (RobotContainer.getSwerve().getPose().getX() < Units.feetToMeters(21) && !OdometryMath2023.isBlue())) {
            if (arm.getAngle() > (Math.PI / 8)) {
                elevator.setHeight(setPoint.getHeight());
            }    
        } else {
            if (arm.getAngle() > -(Math.PI / 6)) {
                elevator.setHeight(setPoint.getHeight());
            }
        }

        arm.extend(setPoint.getExtended());
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Elevator Error: " + (elevator.getHeight() - setPoint.getHeight()) + "   Arm Error: " + (arm.getAngle() - setPoint.getAngle()));

        if (RobotContainer.getJoy().getHID().getRawButton(8)) {
            return false;
        } else if (elevator.atGoal(setPoint.getHeight()) && arm.atGoal(setPoint.getAngle())) {
            goodToRelease = true;
            return true;
        } else if (setPoint.equals(RoutineConstants.DEFAULT_CLAW_STATE) && elevator.atGoal(setPoint.getHeight()) && arm.atJankGoal()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.post("Arm Finished", true);
        timer.stop();
        timer.reset();
    }
}
