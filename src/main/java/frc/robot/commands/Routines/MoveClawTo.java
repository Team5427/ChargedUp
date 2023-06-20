package frc.robot.commands.Routines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.Routines.StateTypes.ClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.util.OdometryMath2023;

public class MoveClawTo extends CommandBase {
    
    private ClawState setPoint;
    private Timer timer;
    private Arm arm;
    private Elevator elevator;
    private Intake intake;
    public static boolean isRunning;
    public static boolean goodToRelease;

    public MoveClawTo(ClawState setPoint) {
        this.setPoint = setPoint;
        arm = RobotContainer.getArm();
        elevator = RobotContainer.getElevator();
        intake = RobotContainer.getIntake();
        timer = new Timer();
        isRunning = false;
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        goodToRelease = false;
        arm.resetPIDs();
        elevator.resetPIDs();
        timer.reset();
        timer.start();
        isRunning = true;
    }

    @Override
    public void execute() {
        intake.setRetracted(setPoint.getRetracted());
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
        if (elevator.atGoal(setPoint.getHeight()) && arm.atGoal(setPoint.getAngle())) {
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
        System.out.println("move claw to just finished");
        
        isRunning = false;
        timer.stop();
        timer.reset();
    }
}
