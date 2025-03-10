package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoAlgaeL2 extends SequentialCommandGroup {
        public AutoAlgaeL2(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem,
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                // First reset heading if possible to note that we are at -90
                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(() -> elevatorSubsystem
                                                                                .elevatorAtAlgaeReefL2(false)),
                                                                new PrintCommand("elevator at Algae L2")),
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(() -> pivotSubsystem
                                                                                .pivotAtAlgaeReef2DropOffAngle(
                                                                                                false)),
                                                                new PrintCommand("pivot at algae L2"))),
                                new ParallelDeadlineGroup(
                                                // Go to Algae L2 alt, turn on ()
                                                new AutoRemoveAlgaeL2(elevatorSubsystem, pivotSubsystem, algaeSubsystem,
                                                                false),
                                                // Move straight to reef (calculate distance (change -1 to correct
                                                // distance), better to run a little
                                                // farther than not enough)
                                                new SequentialCommandGroup(
                                                                new AutoDriveMeters(driveSubsystem, 0, 1.7, 0.4),
                                                                new AutoDriveMeters(driveSubsystem, 0, .5, 0.2))),
                                // Wait X time until algae has cleared the reef
                                new WaitCommand(1),
                                // Go to Coral Drop Off 3
                                new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, false),
                                // Wait until it's at the correct height and angle for L3
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff3Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(false)),
                                                                new PrintCommand("At Pivot Drop Off"))));
        }
}