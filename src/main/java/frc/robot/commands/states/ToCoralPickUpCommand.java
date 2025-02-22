package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralPickUpCommand extends SequentialCommandGroup {
    public ToCoralPickUpCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            WindmillState prevState) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafefromHomeToPickUp(elevatorSubsystem, prevState),
                // Move pivot
                new PivotToCoralPickUp(pivotSubsystem),
                // Move elevator
                new ElevatorToCoralPickUp(elevatorSubsystem));
        // Switch to target state.
    }
}