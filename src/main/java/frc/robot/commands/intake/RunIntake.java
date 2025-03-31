package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class RunIntake extends Command {
    private final boolean m_eject;
    private final IntakeSubsystem m_intake;

    public RunIntake(IntakeSubsystem intake, boolean eject) {
        this.m_intake = intake;
        this.m_eject = eject;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (this.m_eject) {
            this.m_intake.ejectRollers();
        } else {
            this.m_intake.enableRollers();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}