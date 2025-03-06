package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetYellow extends SequentialCommandGroup {
    public ToAlgaeNetYellow(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors
                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaeNetYellow, false)
        // Switch to target state.
        );
    }
}