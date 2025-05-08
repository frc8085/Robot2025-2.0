package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.EndEffector.EndConstants;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class PickupCoral extends Command {

    private final IntakeSubsystem m_intake;
    private boolean isFinished = false;

    public PickupCoral(IntakeSubsystem intake) {
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
    public void execute() {
        if (m_intake.hasCoralCentered()) {
            // wait for both light sensors to be detected
            m_intake.disableRollers();
            System.out.println("Coral Centered");
            this.isFinished = true;
        } else if (m_intake.getAnyLightSensor()) {
            new CenterCoral(m_intake);
        }
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableRollers();
    }
}