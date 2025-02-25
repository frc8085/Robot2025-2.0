package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetLeftCommand extends SequentialCommandGroup {
    public ToAlgaeNetLeftCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafe(elevatorSubsystem),
                // Move pivot
                new PivotToAlgaeNetLeft(pivotSubsystem),
                // Move elevator
                new ElevatorToAlgaeNetLeft(elevatorSubsystem));
        // Switch to target state.
    }
}