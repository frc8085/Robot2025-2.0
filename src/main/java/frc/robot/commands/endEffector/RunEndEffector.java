package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.EndEffector.EndConstants;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;

public class RunEndEffector extends Command {
    private final boolean m_eject;
    private final EndEffectorSubsystem m_end;

    private boolean isFinished = false;

    public RunEndEffector(EndEffectorSubsystem end, boolean eject) {
        this.m_end = end;
        this.m_eject = eject;
        addRequirements(end);
    }

    @Override
    public void initialize() {
        this.isFinished = false;
        if (this.m_eject) {
            this.m_end.eject();
        } else {
            this.m_end.pickup();
        }
    }

    @Override
    public void execute() {
        if (this.m_eject) {
            // wait for the end effector current to be low
            if (this.m_end.getCurrent() < EndConstants.kEndEffectCurrentLimitEject) {
                this.isFinished = true;
            }
        } else {
            // wait for the end effector current to be high
            if (this.m_end.getCurrent() > EndConstants.kEndEffectCurrentLimitHandOff) {
                this.isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_end.stop();
    }
}