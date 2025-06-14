package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.EndEffector.EndConstants;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class PivotAtHandoff extends Command {

    private final PivotSubsystem m_end;

    public PivotAtHandoff(PivotSubsystem end) {
        this.m_end = end;
        addRequirements(end);
    }

    @Override
    public void initialize() {
        // this.m_end.
    }

    @Override
    public boolean isFinished() {
        return this.m_end.pivotAtHomeAngle();
    }

    @Override
    public void end(boolean interrupted) {

    }
}