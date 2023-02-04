package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.commands.Routines.MoveBotTo;
import frc.robot.commands.Routines.MoveClawTo;

public class ButtonBindings {
    public ButtonBindings(CommandJoystick joy) {
        joy.button(3).onTrue(new MoveBotTo(RoutineConstants.POSITION_TYPE.LEFT_CONE));
        joy.button(4).onTrue(new MoveClawTo(RoutineConstants.SUBSTATION_CLAW_STATE));
    }
}