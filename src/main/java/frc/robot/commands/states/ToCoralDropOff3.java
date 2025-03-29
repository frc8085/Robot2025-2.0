package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class ToCoralDropOff3 extends SequentialCommandGroup {
    public ToCoralDropOff3(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean yellow) {
        if (yellow) {
            addCommands(
                    new PrintCommand("Move to Y Coral Drop Off 3"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff3,
                            true));
        } else {
            addCommands(
                    new PrintCommand("Move to B Coral Drop Off 3"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff3,
                            false));
        }
    }
}