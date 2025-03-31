package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ToAlgaeNet extends SequentialCommandGroup {
    public ToAlgaeNet(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean yellow) {
        if (yellow) {
            addCommands(
                    new PrintCommand("Move to Y Algae Net"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaeNetYellow,
                            false));
        } else {
            addCommands(
                    new PrintCommand("Move to B Algae Net"),
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaeNetBlue,
                            false));

        }

    }
}