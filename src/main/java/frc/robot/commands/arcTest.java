package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class arcTest extends Command {
    PivotSubsystem pivotArm;

    public arcTest(PivotSubsystem pivotArm) {
        this.pivotArm = pivotArm;
        addRequirements(pivotArm);

    }

    @Override
    public void initialize() {

    }

    public void execute() {
        pivotArm.setPos(Rotation2d.fromRotations(0));

    }
}
