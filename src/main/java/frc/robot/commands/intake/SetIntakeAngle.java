package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SetIntakeAngle extends Command {
    private final Rotation2d m_targetAngle;
    private final IntakeSubsystem m_intake;
    private final Rotation2d tolerance = IntakeConstants.kIntakeTolerance;

    public SetIntakeAngle(IntakeSubsystem intake, Rotation2d targetAngle) {
        m_targetAngle = targetAngle;
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setDeployRotation(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return m_intake.isAtIntakeAngle(tolerance);
    }
}