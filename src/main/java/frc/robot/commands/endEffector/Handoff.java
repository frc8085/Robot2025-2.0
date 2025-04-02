package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.EjectCoral;

public class Handoff extends SequentialCommandGroup {
        public Handoff(
                        IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem) {

                addCommands(
                                new PrintCommand("Handoff 1"),
                                new RunEndEffector(endEffectorSubsystem, false),
                                new PrintCommand("Handoff 2"),
                                new InstantCommand(() -> endEffectorSubsystem.pickup()),
                                new PrintCommand("Handoff 3"),
                                new WaitCommand(0.2),
                                new PrintCommand("Handoff 4"),
                                new EjectCoral(intakeSubsystem),
                                new PrintCommand("Handoff 5"),
                                new InstantCommand(() -> endEffectorSubsystem.stop()));
        }
}