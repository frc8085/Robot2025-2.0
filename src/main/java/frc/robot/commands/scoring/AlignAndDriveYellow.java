package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignAndDriveYellow extends SequentialCommandGroup {
    public AlignAndDriveYellow(DriveSubsystem drive, LimelightSubsystem limelight) {
        addCommands(
                new AlignToAprilTagYellow(drive, limelight),
                new DriveToReefYellow(drive, limelight),
                new AlignToAprilTagYellow(drive, limelight));

    }

}
