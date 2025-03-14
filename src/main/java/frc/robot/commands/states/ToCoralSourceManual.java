package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralSourceManual extends SequentialCommandGroup {
    public ToCoralSourceManual(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean altButton) {
        if (altButton) {
            addCommands(
                    new PrintCommand("Move to Coral Pick Up Alt Manual"),
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.CoralPickupAlternate,
                            false));
        } else {
            addCommands(
                    new PrintCommand("Move to Coral Pick Up Manual"),
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.CoralPickup,
                            false));

        }
    }

}
