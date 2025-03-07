package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagBlue;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.AutoPositionLeftRight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightPosition extends SequentialCommandGroup {
    public AutoLimelightPosition(DriveSubsystem drive, LimelightSubsystem limelight, boolean right, boolean yellow) {
        if (yellow) {
            addCommands(
                    new AutoDriveToReefYellow(drive, limelight),
                    new AlignToAprilTagYellow(drive, limelight),
                    new AutoPositionLeftRight(drive, limelight, right, yellow));
        } else {
            addCommands(
                    new AutoDriveToReefBlue(drive, limelight),
                    new AlignToAprilTagBlue(drive, limelight),
                    new AutoPositionLeftRight(drive, limelight, right, yellow));
        }

    }
}
