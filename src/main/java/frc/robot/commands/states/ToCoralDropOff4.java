package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralDropOff4 extends SequentialCommandGroup {
    public ToCoralDropOff4(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean yellow) {
        if (yellow) {
            addCommands(
                    new PrintCommand("Move to Y Coral Drop Off 4"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff4,
                            true));
        } else {
            addCommands(
                    new PrintCommand("Move to B Coral Drop Off 4"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff4,
                            false));
        }
    }
}