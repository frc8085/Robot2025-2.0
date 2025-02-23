package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToAlgaeNetRight extends Command {
    PivotSubsystem pivot;

    public PivotToAlgaeNetRight(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPos(WindmillState.AlgaeNetRight.getPivotArmAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivot.getCurrentPosition()
                - WindmillState.AlgaeNetRight.getPivotArmAngle()
                        .getRotations()) < Constants.PivotArmConstants.kPivotToleranceRotations; // TODO Switch to using
                                                                                                 // the Rotation2d value
    }
}