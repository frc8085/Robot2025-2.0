package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CenterCoral extends Command {
    private final IntakeSubsystem m_intake;
    // private boolean isFinished = false;

    public CenterCoral(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // this.isFinished = false;
        m_intake.setDeployRotation(IntakeConstants.kIntakeInAngle);
        m_intake.disableOuterRollers();

        if (this.m_intake.hasCoralCentered()) {
            // this.isFinished = true;
            return;
        } else {
            m_intake.idleRollers();
        }
    }

    @Override
    public boolean isFinished() {
        // check if both light sensors are triggered
        return this.m_intake.hasCoralCentered();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableRollers();
    }
}