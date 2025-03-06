package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToScoreYellow extends SequentialCommandGroup {
    public ToScoreYellow(DriveSubsystem drive, CoralSubsystem coral, AlgaeSubsystem algae, PivotSubsystem pivot,
            ElevatorSubsystem elevator, boolean yellow) {
        addCommands(
                new MoveToScoreYellow(drive));
        // new ToAlgaeScoreYellow(algae, elevator, pivot, drive, true),
        // new ToCoralScoreYellow(coral, elevator, pivot, drive),
        // new ResetOperatorInputs());
    }

}