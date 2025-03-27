package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.manipulator.algae.EjectAlgae;
import frc.robot.commands.states.ToAlgaeNet;
import frc.robot.commands.states.ToAlgaeNetRaise;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class AutoScoreAlgaeNetBlue extends SequentialCommandGroup {
        public AutoScoreAlgaeNetBlue(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem) {
                addCommands(
                                new ToAlgaeNetRaise(elevatorSubsystem, pivotSubsystem),
                                // new AngleToBarge(driveSubsystem),
                                new ToAlgaeNet(elevatorSubsystem, pivotSubsystem, false),
                                new WaitUntilCommand(pivotSubsystem::pivotAtAlgaeBlueScorePosition),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtAlgaeScoreHeight),
                                new EjectAlgae(algaeSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem));

        }
}
