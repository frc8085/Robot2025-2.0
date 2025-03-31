package frc.robot.commands.windmill.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Pivot.PivotArmConstants;

public class Pivot extends Command {
    private final Rotation2d m_targetAngle;
    private final PivotSubsystem m_pivot;
    private final Rotation2d tolerance = PivotArmConstants.kPivotTolerance;

    public Pivot(PivotSubsystem pivot, Rotation2d targetAngle) {
        m_targetAngle = targetAngle;
        m_pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setPos(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isAtTarget(tolerance);
    }
}