package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorDebug extends CommandBase {

    CommandJoystick joy;
    Elevator elevator;
    double elevatorSpeed;


    public ElevatorDebug() {
        elevator = RobotContainer.getElevator();
        joy = RobotContainer.getPilotJoy();
        elevatorSpeed = .15;

        addRequirements(elevator);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(joy.getHID().getRawButton(JoystickConstants.debugElevatorUp) 
        // && elevator.getHeight() < ElevatorConstants.UPPER_LIMIT_METERS
        ){
            elevator.set(elevatorSpeed);
        } else if(joy.getHID().getRawButton(JoystickConstants.debugElevatorDown)
        // && !elevator.atLowerLimit()
        ){
            elevator.set(-elevatorSpeed);
        } else{
            elevator.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted){
        elevator.stop();
    }
}
