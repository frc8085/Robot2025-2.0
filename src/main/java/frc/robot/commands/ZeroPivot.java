package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PivotSubsystem;

public class ZeroPivot extends
        SequentialCommandGroup {

    public ZeroPivot(PivotSubsystem pivotSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> pivotSubsystem.setRotorPos(Rotation2d.fromDegrees(0))),
                        new InstantCommand(() -> pivotSubsystem.setPos(Rotation2d.fromDegrees(0)))));
    }
}