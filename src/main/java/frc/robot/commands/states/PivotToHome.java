package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToHome extends Command {
    PivotSubsystem pivot;

    public PivotToHome(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPos(WindmillState.Home.getPivotArmAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivot.getCurrentPosition()
                - WindmillState.Home.getPivotArmAngle()
                        .getRotations()) < Constants.PivotArmConstants.kPivotToleranceRotations; // TODO Switch to using
                                                                                                 // the Rotation2d value
    }
}