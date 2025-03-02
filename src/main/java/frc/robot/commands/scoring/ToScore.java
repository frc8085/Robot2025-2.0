package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.commands.scoring.*;

public class ToScore extends SequentialCommandGroup {
    public ToScore(DriveSubsystem drive, CoralSubsystem coral, AlgaeSubsystem algae, PivotSubsystem pivot) {
        addCommands(
                new MoveToScore(drive),
                new ToAlgaeScore(algae, pivot),
                new ToCoralScore(coral, pivot),
                new ResetOperatorInputs());
    }

}