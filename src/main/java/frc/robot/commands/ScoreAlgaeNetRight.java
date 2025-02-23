package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.states.ToAlgaeNetRightCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreAlgaeNetRight extends SequentialCommandGroup {
        public ScoreAlgaeNetRight(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem) {
                addCommands(
                                new ToAlgaeNetRightCommand(elevatorSubsystem, pivotSubsystem),
                                new ScoreAlgae(algaeSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, WindmillState.AlgaeNetRight,
                                                coralSubsystem));

        }
}
