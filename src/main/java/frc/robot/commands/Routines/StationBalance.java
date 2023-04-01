// package frc.robot.commands.Routines;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Routines.Balancing.BalanceLinear;
// // import frc.robot.commands.Routines.BasicMovement.TurnAndTranslate;

// public class StationBalance extends SequentialCommandGroup {
//     public StationBalance(boolean fromFront, boolean facingFront) {

//         Rotation2d heading;
//         Rotation2d movementDirection;

//         if (fromFront) {
//             movementDirection = new Rotation2d(Math.PI);
//         } else {
//             movementDirection = new Rotation2d(0);
//         }

//         if (facingFront) {
//             heading = new Rotation2d(0);
//         } else {
//             heading = new Rotation2d(Math.PI);
//         }

//         addCommands(
//             new TurnAndTranslate(heading, movementDirection, 2.5, 2.1),
//             new BalanceLinear()
//         );
//     }
// }
