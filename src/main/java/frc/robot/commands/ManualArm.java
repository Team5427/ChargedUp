// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.JoystickConstants;
// import frc.robot.subsystems.Arm;

// public class ManualArm extends CommandBase{
//     private Arm arm;
//     private double speed;

//     public ManualArm(double speed){
//         arm = RobotContainer.getArm();
//         this.speed = speed;
//     }

//     @Override
//     public void initialize(){

//     }

//     @Override
//     public void execute(){
//         arm.set(speed);
//     }

//     @Override
//     public boolean isFinished(){
//         return !RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.ARM_RESET);
//     }

//     @Override
//     public void end(boolean interrupted){
//         arm.stop();
//     }
    
// }
