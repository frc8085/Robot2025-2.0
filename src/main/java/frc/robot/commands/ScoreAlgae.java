package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;

public class ScoreAlgae extends SequentialCommandGroup {

        public ScoreAlgae(
                        AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                new ParallelDeadlineGroup(new WaitCommand(0.5),
                                                new InstantCommand(algaeSubsystem::eject)),
                                new InstantCommand(algaeSubsystem::stop));
        }

}
