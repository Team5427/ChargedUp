package frc.robot.commands.Routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Routines.StateTypes.ClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveClawTo extends CommandBase {
    
    private ClawState setPoint;
    private Timer timer;
    private Arm arm;
    private Elevator elevator;

    public MoveClawTo(ClawState setPoint) {
        this.setPoint = setPoint;
        arm = RobotContainer.getArm(); //commenting out since havent implemented in robotcontainer yet
        elevator = RobotContainer.getElevator();
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
        arm.setAngle(setPoint.getAngle());
        if (arm.getAngle() > (Math.PI / 16)) {
            elevator.setHeight(setPoint.getHeight());
        }

        arm.extend(setPoint.getExtended());
    }

    @Override
    public boolean isFinished() {
        if (RobotContainer.getJoy().getHID().getRawButton(8)) {
            return false;
        } else if (elevator.atGoal(setPoint.getHeight()) && arm.atGoal(setPoint.getAngle())) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }
}
