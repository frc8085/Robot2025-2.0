package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoPosition extends SequentialCommandGroup {
    public AutoPosition(DriveSubsystem drive, LimelightSubsystem limelight, boolean right) {
        if (right) {
            if (limelight.hasTarget("limelight-yellow")) {
                addCommands(
                        // move right for yellow
                        new AutoDriveMeters(drive, 0.17, 0, 0.1));
            } else {
                addCommands(
                        // move right for blue
                        new AutoDriveMeters(drive, 0.17, 0, 0.1));

            }
        } else {
            if (limelight.hasTarget("limelight-yellow")) {
                addCommands(
                        // move left for yellow
                        new AutoDriveMeters(drive, -0.17, 0, 0.1));
            } else {
                addCommands(
                        // move left for blue
                        new AutoDriveMeters(drive, -0.17, 0, 0.1));

            }

        }

    }

}
