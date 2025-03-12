package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.manipulator.coral.*;
import frc.robot.commands.manipulator.algae.*;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RemoveAlgaeL3ScoreL3 extends SequentialCommandGroup {
        public RemoveAlgaeL3ScoreL3(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
                        boolean yellow) {
                if (yellow) {
                        addCommands(
                                        new ParallelCommandGroup(
                                                        new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, yellow),
                                                        new AutoDriveMeters(driveSubsystem, -0.1, 0, 0.1)),
                                        new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                        new WaitUntilCommand(() -> elevatorSubsystem
                                                                                        .elevatorAtAlgaeReefL3(yellow)),
                                                                        new PrintCommand("elevator at Algae L3")),
                                                        new SequentialCommandGroup(
                                                                        new WaitUntilCommand(() -> pivotSubsystem
                                                                                        .pivotAtAlgaeReef3DropOffAngle(
                                                                                                        yellow)),
                                                                        new PrintCommand("pivot at algae L3"))),
                                        new ParallelDeadlineGroup(
                                                        new PickUpAlgae(algaeSubsystem),
                                                        new AutoDriveMeters(driveSubsystem, 0, -0.1, 0.1)),
                                        new WaitCommand(.25),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.Home,
                                                        false),
                                        new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, yellow),
                                        new ParallelCommandGroup(
                                                        new WaitUntilCommand(
                                                                        elevatorSubsystem::elevatorAtCoralDropOff3Height),
                                                        new WaitUntilCommand(() -> pivotSubsystem
                                                                        .pivotAtCoralDropOffAngle(yellow))),
                                        new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                } else {
                        addCommands(
                                        new ParallelCommandGroup(
                                                        new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, yellow),
                                                        new AutoDriveMeters(driveSubsystem, 0.1, 0, 0.1)),
                                        new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                        new WaitUntilCommand(() -> elevatorSubsystem
                                                                                        .elevatorAtAlgaeReefL3(yellow)),
                                                                        new PrintCommand("elevator at Algae L3")),
                                                        new SequentialCommandGroup(
                                                                        new WaitUntilCommand(() -> pivotSubsystem
                                                                                        .pivotAtAlgaeReef3DropOffAngle(
                                                                                                        yellow)),
                                                                        new PrintCommand("pivot at algae L3"))),
                                        new ParallelDeadlineGroup(
                                                        new PickUpAlgae(algaeSubsystem),
                                                        new AutoDriveMeters(driveSubsystem, 0, 0.1, 0.1)),
                                        new WaitCommand(.25),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.Home,
                                                        false),
                                        new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, yellow),
                                        new ParallelCommandGroup(
                                                        new WaitUntilCommand(
                                                                        elevatorSubsystem::elevatorAtCoralDropOff3Height),
                                                        new WaitUntilCommand(() -> pivotSubsystem
                                                                        .pivotAtCoralDropOffAngle(yellow))),
                                        new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));

                }
        }
}