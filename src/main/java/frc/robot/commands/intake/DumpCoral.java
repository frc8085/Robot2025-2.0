package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class DumpCoral extends SequentialCommandGroup {
    public DumpCoral(
            IntakeSubsystem intakeSubsystem) {
        addCommands(
                new SetIntakeAngle(intakeSubsystem, IntakeConstants.kIntakeEjectAngle),
                new EjectCoral(intakeSubsystem));
    }
}
