package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

/*Turn Off Intake Motors */
public class IntakeMotorsOff extends ParallelCommandGroup {
        public IntakeMotorsOff(
                        CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                // Turn off Coral and Algae Motors
                                new RunCommand(() -> coralSubsystem.stop(), coralSubsystem).withTimeout(0.25),
                                new RunCommand(() -> algaeSubsystem.stop(), algaeSubsystem).withTimeout(0.25));

        }
}
