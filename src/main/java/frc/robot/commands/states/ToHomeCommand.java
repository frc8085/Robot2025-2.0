package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToHomeCommand extends SequentialCommandGroup {
    public ToHomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState prevState) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafefromPickUpToHome(elevatorSubsystem, prevState),
                // Move pivot
                new PivotToHome(pivotSubsystem),
                // Move elevator
                new ElevatorToHome(elevatorSubsystem));
        // Switch to target state.
    }
}