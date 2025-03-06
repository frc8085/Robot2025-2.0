package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignAndDriveBlue extends SequentialCommandGroup {
    public AlignAndDriveBlue(DriveSubsystem drive, LimelightSubsystem limelight) {
        addCommands(
                new AlignToAprilTagBlue(drive, limelight),
                new DriveToReefBlue(drive, limelight),
                new AlignToAprilTagBlue(drive, limelight));

    }

}
