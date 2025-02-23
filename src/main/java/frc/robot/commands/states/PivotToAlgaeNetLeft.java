package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToAlgaeNetLeft extends Command {
    PivotSubsystem pivot;

    public PivotToAlgaeNetLeft(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPos(WindmillState.AlgaeNetLeft.getPivotArmAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivot.getCurrentPosition()
                - WindmillState.AlgaeNetLeft.getPivotArmAngle()
                        .getRotations()) < Constants.PivotArmConstants.kPivotToleranceRotations; // TODO Switch to using
                                                                                                 // the Rotation2d value
    }
}