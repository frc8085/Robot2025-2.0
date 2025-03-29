package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class ToAlgaeL3 extends SequentialCommandGroup {
    public ToAlgaeL3(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean yellow) {
        if (yellow) {
            addCommands(
                    new PrintCommand("Move to Y Algae L3"),
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.AlgaePickUpReef3Flip,
                            false));
        } else {
            addCommands(
                    new PrintCommand("Move to B Algae L3"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaePickUpReef3,
                            false));
        }

    }
}