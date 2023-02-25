package frc.robot.commands.Routines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RampPusher;

public class MoveClawTo extends CommandBase {
    
    private ClawState setPoint;
    private Timer timer;
    private Arm arm;
    private Elevator elevator;
    private RampPusher pusher;

    public MoveClawTo(ClawState setPoint) {
        this.setPoint = setPoint;
        arm = RobotContainer.getArm(); //commenting out since havent implemented in robotcontainer yet
        elevator = RobotContainer.getElevator();
        pusher = RobotContainer.getRampPusher();
        timer = new Timer();
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (setPoint.getAngle() < (Math.PI / 6) && setPoint.getHeight() < 0.12) {
            pusher.deploy(true);
        }
        if (pusher.atSetpoint()) {
            arm.setAngle(setPoint.getAngle());
            if (arm.getAngle() > 0) {
                elevator.setHeight(setPoint.getHeight());
            }
    
            arm.extend(setPoint.getExtended());
        }
    }

    @Override
    public boolean isFinished() {
        if (RobotContainer.getPilotJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_P) || RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O)) {
            return true;
        } else if (elevator.atGoal() && arm.atGoal()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (setPoint.getAngle() > (Math.PI / 6) || setPoint.getHeight() > 0.12) {
            pusher.deploy(false);
        }
        timer.stop();
        timer.reset();
    }
}
