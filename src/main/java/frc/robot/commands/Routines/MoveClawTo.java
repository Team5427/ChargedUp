package frc.robot.commands.Routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RoutineConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveClawTo extends CommandBase {
    
    private ClawState setPoint;
    private Timer timer;
    private Arm arm;
    private Elevator elevator;

    public MoveClawTo(ClawState setPoint) {
        this.setPoint = setPoint;
        // arm = RobotContainer.getArm(); //commenting out since havent implemented in robotcontainer yet
        // elevator = RobotContainer.getElevator();
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
        elevator.setHeight(setPoint.getHeight());
        if (timer.get() >= RoutineConstants.ARM_DELAY_SECONDS) {
            arm.setAngle(setPoint.getAngle());
        }

        if (arm.atGoal()) {
            arm.extend(setPoint.getExtended());
        }
    }

    @Override
    public boolean isFinished() {
        return (elevator.atGoal() && arm.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }
}
