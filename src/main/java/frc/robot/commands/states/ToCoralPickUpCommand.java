package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralPickUp extends SequentialCommandGroup {
    public ToCoralPickUp(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState prevState) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafefromHomeToPickUp(elevatorSubsystem, prevState),
                // Move pivot
                new PivotToHome(pivotSubsystem),
                // Move elevator
                new ElevatorToHome(elevatorSubsystem));
        // Switch to target state.
    }
}