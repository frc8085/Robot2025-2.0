package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToCoralDropOff4 extends Command {
    PivotSubsystem pivot;

    public PivotToCoralDropOff4(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPos(WindmillState.CoralDropOff4.getPivotArmAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivot.getCurrentPosition()
                - WindmillState.CoralDropOff4.getPivotArmAngle()
                        .getRotations()) < Constants.PivotArmConstants.kPivotToleranceRotations; // TODO Switch to using
                                                                                                 // the Rotation2d value
    }
}