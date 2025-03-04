package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlgaeLevel;
import frc.robot.commands.PickUpAlgaeAndReturnToHome;
// import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

//TO DO: Need to add which limelight we're using

public class ToAlgaeScore extends SequentialCommandGroup {
    public ToAlgaeScore(AlgaeSubsystem algae, ElevatorSubsystem elevator, PivotSubsystem pivot, DriveSubsystem drive) {
        switch (RobotContainer.algaeLevel) {
            // Add the specific commands in states in here.
            case TWO:
                addCommands(
                        new ScoringMoveToAlgaePickup(algae, elevator, pivot, false,
                                false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtAlgaeReefL2),
                                new WaitUntilCommand(pivot::pivotAtAlgaeReefDropOffAngle)),
                        // new AutoDriveMeters(drive, 0.05, 0),
                        new PickUpAlgaeAndReturnToHome(algae, elevator, pivot));
                break;
            case THREE:
                addCommands(
                        new ScoringMoveToAlgaePickup(algae, elevator, pivot, true, false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtAlgaeReefL3),
                                new WaitUntilCommand(pivot::pivotAtAlgaeReefDropOffAngle)),
                        // new AutoDriveMeters(drive, 0.05, 0),
                        new PickUpAlgaeAndReturnToHome(algae, elevator, pivot));
                break;
            case NONE:
                break;
            case UNDECIDED:
                addCommands(
                        // Waits until a level is given and will rerun this current command
                        // TODO: Add a way to exit this command in case you don't want to score or
                        // change your mind
                        new WaitUntilCommand(
                                new BooleanSupplier() {
                                    @Override
                                    public boolean getAsBoolean() {
                                        return RobotContainer.algaeLevel != AlgaeLevel.UNDECIDED;
                                    }
                                }),
                        new ToAlgaeScore(algae, elevator, pivot, drive));
                break;
        }
    }
}