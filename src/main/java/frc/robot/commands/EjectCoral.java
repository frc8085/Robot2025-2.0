package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralSubsystem;

public class EjectCoral extends SequentialCommandGroup {
        public EjectCoral(
                        CoralSubsystem coralSubsystem) {
                addCommands(

                                new InstantCommand(coralSubsystem::eject),
                                new WaitCommand(1),
                                new InstantCommand(coralSubsystem::stop));

        }
}
