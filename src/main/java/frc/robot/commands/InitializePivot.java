package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.PivotSubsystem;

public class InitializePivot extends
                SequentialCommandGroup {

        public InitializePivot(PivotSubsystem pivotSubsystem) {
                addCommands(
                                new SequentialCommandGroup(
                                                new ConditionalCommand(new InstantCommand(pivotSubsystem::zeroStart),
                                                                new InstantCommand(pivotSubsystem::zeroReverse),
                                                                (pivotSubsystem::lessThanHomeAngle)),
                                                new WaitUntilCommand(pivotSubsystem::atHomeAngle)),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> pivotSubsystem
                                                                .setAnglePos(Rotation2d
                                                                                .fromDegrees(45))),
                                                new InstantCommand(() -> pivotSubsystem
                                                                .setPos(Rotation2d.fromDegrees(45)))));
        }
}