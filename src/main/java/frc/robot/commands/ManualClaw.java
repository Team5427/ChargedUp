// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.JoystickConstants;
// import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Led;

// public class ManualClaw extends CommandBase{
//     private Claw claw;
//     private double speed;

//     public static boolean isRunning;

//     public ManualClaw(double speed){
//         claw = RobotContainer.getClaw();
//         this.speed = speed;
//         isRunning = false;
//     }

//     @Override
//     public void initialize(){
//         isRunning = true;
//     }

//     @Override
//     public void execute(){
//         claw.set(speed);
//     }

//     @Override
//     public boolean isFinished(){
//         return (!RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CLAW_INTAKE) && 
//         !RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CLAW_OUTTAKE));
//     }

//     @Override
//     public void end(boolean interrupted){

//         if(claw.proxCovered()){
//             RobotContainer.getLed().setState(Led.SCORING);
//         } else{
//             RobotContainer.getLed().setState(Led.INTAKE);
//         }
//         isRunning = false;
//         claw.stop();
//     }
    
// }
