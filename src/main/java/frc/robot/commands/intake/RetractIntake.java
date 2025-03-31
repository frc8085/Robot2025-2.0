package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class RetractIntake extends SequentialCommandGroup {
    public RetractIntake(
            IntakeSubsystem intakeSubsystem) {
        addCommands(
                new SetIntakeAngle(intakeSubsystem, IntakeConstants.kIntakeInAngle),
                new StopIntake(intakeSubsystem));
    }
}
