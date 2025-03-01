package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;

public class ScoreAlgae extends SequentialCommandGroup {

        public ScoreAlgae(
                        AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                new RunCommand(() -> algaeSubsystem.eject(), algaeSubsystem).withTimeout(0.5),
                                new InstantCommand(algaeSubsystem::stop));
        }

}
