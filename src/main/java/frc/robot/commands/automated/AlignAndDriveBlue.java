package frc.robot.commands.automated;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagBlue;
import frc.robot.commands.movement.DriveToReefBlue;
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
