package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetBlue extends SequentialCommandGroup {
    public ToAlgaeNetBlue(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaeNetBlue, false)

        );

    }
}