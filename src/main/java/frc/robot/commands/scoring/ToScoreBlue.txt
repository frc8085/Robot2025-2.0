package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToScoreBlue extends SequentialCommandGroup {
    public ToScoreBlue(DriveSubsystem drive, CoralSubsystem coral, AlgaeSubsystem algae, PivotSubsystem pivot,
            ElevatorSubsystem elevator, boolean yellow) {
        addCommands(
                new MoveToScoreBlue(drive));
        // new ToAlgaeScoreBlue(algae, elevator, pivot, drive, yellow),
        // new ToCoralScoreBlue(coral, elevator, pivot, drive),
        // new ResetOperatorInputs());
    }

}