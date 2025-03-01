package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetLeftCommand extends SequentialCommandGroup {
    public ToAlgaeNetLeftCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaeNetLeft, false)

        );

    }
}