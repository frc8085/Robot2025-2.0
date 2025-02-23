package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralDropOff4 extends SequentialCommandGroup {
    public ToCoralDropOff4(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafe(elevatorSubsystem),
                // Move pivot
                new PivotToCoralDropOff4(pivotSubsystem),
                // Move elevator
                new ElevatorToCoralDropOff4(elevatorSubsystem));
        // Switch to target state.
    }
}