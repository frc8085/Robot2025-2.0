package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.states.ToAlgaeNetLeftCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreAlgaeNetLeft extends SequentialCommandGroup {
        public ScoreAlgaeNetLeft(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem) {
                addCommands(
                                new ToAlgaeNetLeftCommand(elevatorSubsystem, pivotSubsystem),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtAlgaeScoreHeight),
                                new ScoreAlgae(algaeSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem,
                                                coralSubsystem));

        }
}
