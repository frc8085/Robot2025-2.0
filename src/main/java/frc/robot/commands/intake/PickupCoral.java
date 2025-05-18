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

    @Override
    public void initialize() {

        this.isFinished = false;
        if (m_intake.hasCoralCentered()) {
            System.out.println("Coral Already Centered");
            this.isFinished = true;
        } else if (m_intake.getAnyLightSensor()) {
            return;
        } else {
            m_intake.setDeployRotation(IntakeConstants.kIntakeOutAngle);
            m_intake.enableOuterRollers();
        }
    }

    @Override
    public void execute() {
        if (m_intake.hasCoralCentered()) {
            // wait for both light sensors to be detected
            m_intake.disableRollers();
            System.out.println("Coral Centered");
            this.isFinished = true;
        } else if (m_intake.getAnyLightSensor()) {
            m_intake.setDeployRotation(IntakeConstants.kIntakeInAngle);
            m_intake.idleOuterRollers();
            m_intake.idleRollers();
        } else {
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableRollers();
        m_intake.setDeployRotation(IntakeConstants.kIntakeInAngle);
    }
}