package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoPositionLeftRight extends SequentialCommandGroup {
    public AutoPositionLeftRight(DriveSubsystem drive, LimelightSubsystem limelight, boolean right, boolean yellow) {
        if (limelight.hasTarget("limelight-yellow")) {
            yellow = true;
        } else {
            yellow = false;
        }

        if (right) {
            if (yellow) {
                addCommands(
                        // move right for yellow
                        new AutoDriveMeters(drive, 0.17, 0, 0.1));
            } else {
                addCommands(
                        // move right for blue
                        new AutoDriveMeters(drive, -0.17, 0, 0.1));

            }
        } else {
            if (yellow) {
                addCommands(
                        // move left for yellow
                        new AutoDriveMeters(drive, -0.17, 0, 0.1));
            } else {
                addCommands(
                        // move left for blue
                        new AutoDriveMeters(drive, 0.17, 0, 0.1));

            }

        }

    }

}
