package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.PickUpAlgae;
import frc.robot.commands.Windmill;
import frc.robot.commands.movement.AutoMoveForward;
import frc.robot.commands.movement.AutoMoveForwardForTime;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RemoveAlgaeL2 extends SequentialCommandGroup {
        public RemoveAlgaeL2(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
                        LimelightSubsystem limelight,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
                        boolean yellow) {
                addCommands(
                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, yellow),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(() -> elevatorSubsystem
                                                                                .elevatorAtAlgaeReefL2(yellow)),
                                                                new PrintCommand("elevator at Algae L2")),
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(() -> pivotSubsystem
                                                                                .pivotAtAlgaeReef2DropOffAngle(
                                                                                                yellow)),
                                                                new PrintCommand("pivot at algae L2"))),
                                new ParallelRaceGroup(
                                                new PickUpAlgae(algaeSubsystem),
                                                new AutoMoveForwardForTime(driveSubsystem, limelight, yellow,
                                                                2)),
                                new WaitCommand(.25),
                                new AutoMoveForward(driveSubsystem, limelight, yellow),
                                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Home,
                                                false));

        }
}