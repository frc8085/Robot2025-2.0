package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.EjectHandoffCoral;

public class Algae extends SequentialCommandGroup {
        public Algae(EndEffectorSubsystem endEffectorSubsystem) {

                addCommands(

                                new PrintCommand("Handoff 1"),
                                new InstantCommand(() -> endEffectorSubsystem.pickup()),
                                new WaitCommand(2),
                                new PrintCommand("Handoff 2"),
                                new InstantCommand(() -> endEffectorSubsystem.stop()));
        }
}