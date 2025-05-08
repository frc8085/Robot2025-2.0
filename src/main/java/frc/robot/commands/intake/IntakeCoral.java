package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCoral extends Command {

    private final IntakeSubsystem m_intake;
    private boolean isFinished = false;

    public IntakeCoral(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    public void finish() {
        this.isFinished = true;
    }

    @Override
    public void initialize() {

        this.isFinished = false;

        boolean coralCentered = m_intake.hasCoralCentered();
        boolean hasCoral = m_intake.getAnyLightSensor();

        SequentialCommandGroup commands = new SequentialCommandGroup();

        commands.addCommands(new PrintCommand("Starting IntakeCoral"));

        if (coralCentered) {
            commands.addCommands(new PrintCommand("Coral Centered"));
            this.isFinished = true;
        } else if (hasCoral) {
            commands.addCommands(
                    new PrintCommand("Has Coral"),
                    new CenterCoral(m_intake));
        } else {
            commands.addCommands(
                    new PrintCommand("Deploy Intake"),
                    new DeployIntake(m_intake));
        }
        commands.schedule();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        this.isFinished = true;
    }
}