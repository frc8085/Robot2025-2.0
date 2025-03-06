package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.states.ToAlgaeNetYellow;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreAlgaeNetYellow extends SequentialCommandGroup {
        public ScoreAlgaeNetYellow(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem) {
                addCommands(
                                new ToAlgaeNetYellow(elevatorSubsystem, pivotSubsystem),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtAlgaeScoreHeight),
                                new WaitUntilCommand(pivotSubsystem::pivotAtAlgaeYellowScorePosition),
                                new EjectAlgae(algaeSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem,
                                                coralSubsystem));

        }
}
