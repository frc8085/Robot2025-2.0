package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoMoveForward extends SequentialCommandGroup {
    public AutoMoveForward(DriveSubsystem drive, LimelightSubsystem limelight, boolean yellow) {
        if (yellow) {
            addCommands(
                    new AutoDriveMeters(drive, 0, 0.2, 0.2)); // move forward for yellow;
        } else {
            addCommands(
                    new AutoDriveMeters(drive, 0, -0.2, 0.2)); // move forward for blue;

        }
    }
}