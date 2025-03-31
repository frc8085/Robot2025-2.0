package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class StopIntake extends Command {
    private final IntakeSubsystem m_intake;

    public StopIntake(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.m_intake.disableRollers();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}