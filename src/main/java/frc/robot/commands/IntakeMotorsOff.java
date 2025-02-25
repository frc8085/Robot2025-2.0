package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/*Turn Off Intake Motors */
public class IntakeMotorsOff extends SequentialCommandGroup {
        public IntakeMotorsOff(
                        CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                // Turn off Coral and Algae Motors
                                new InstantCommand(coralSubsystem::stop),
                                new InstantCommand(algaeSubsystem::stop));

        }
}
