package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;
import frc.robot.commands.Routines.PositionState;

public class ButtonBindings {
    public ButtonBindings(CommandJoystick joy) {
        joy.button(3).onTrue(new MoveBotTo(PositionState.getScoringPose(RoutineConstants.SCORING_TYPE.BOTTOM_CONE)));
        joy.button(4).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
    }
}