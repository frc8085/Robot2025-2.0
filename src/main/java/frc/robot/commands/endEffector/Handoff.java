package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.EjectCoral;

public class Handoff extends ParallelCommandGroup {
    public Handoff(
            IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem) {

        addCommands(
                new SequentialCommandGroup(
                        new RunEndEffector(endEffectorSubsystem, false),
                        new InstantCommand(() -> endEffectorSubsystem.pickup()),
                        new WaitCommand(0.2),
                        new InstantCommand(() -> endEffectorSubsystem.stop())),
                new EjectCoral(intakeSubsystem));
    }
}