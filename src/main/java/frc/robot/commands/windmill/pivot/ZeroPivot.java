package frc.robot.commands.windmill.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PivotSubsystem;

public class ZeroPivot extends
        SequentialCommandGroup {

    public ZeroPivot(PivotSubsystem pivotSubsystem) {
        addCommands(
                new InstantCommand(() -> pivotSubsystem.setAnglePos(Rotation2d.fromDegrees(90))),
                new InstantCommand(() -> pivotSubsystem.setPos(Rotation2d.fromDegrees(90))));
    }
}