package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/* Command to run at the end of a path movement to check if the coral has been picked up and if not, continue on the auto */
public class CoralCheck extends SequentialCommandGroup {
    public CoralCheck(
            CoralSubsystem coralSubsystem) {
        addCommands(
                // Check if coral has been picked up already, and if it hasn't, wait one
                // sec then end
                new ConditionalCommand(new InstantCommand(),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(coralSubsystem::stop)),
                        coralSubsystem::coralInRobot));

    }
}
