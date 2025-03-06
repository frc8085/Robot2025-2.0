package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.movement.AutoDriveMeters;
// import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToScoreBlue extends Command {
    DriveSubsystem drive;

    public MoveToScoreBlue(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        switch (RobotContainer.scoreDirection) {
            case LEFT:
                // Move to the left
                new AutoDriveMeters(drive, -0.1, 0, 0.1);
                break;
            case RIGHT:
                // Move to the right
                new AutoDriveMeters(drive, 0.1, 0, 0.1);
                break;
            case UNDECIDED:
                // Do nothing until left or right is decided.
                break;
        }
    }
}