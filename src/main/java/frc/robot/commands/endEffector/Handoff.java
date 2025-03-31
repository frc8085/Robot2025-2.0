package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.EjectCoral;

public class Handoff extends ParallelCommandGroup {
    public Handoff(
            IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem) {

        addCommands(
                new RunEndEffector(endEffectorSubsystem, false),
                new EjectCoral(intakeSubsystem));
    }
}