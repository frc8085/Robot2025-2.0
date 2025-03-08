package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagBlue;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.NewAutoMoveOnReef;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightPosition extends SequentialCommandGroup {
    public AutoLimelightPosition(DriveSubsystem drive, LimelightSubsystem limelight, boolean right, boolean yellow) {
        if (yellow) {
            addCommands(
                    new AlignToAprilTagYellow(drive, limelight),
                    new AutoDriveToReefYellow(drive, limelight),
                    new AlignToAprilTagYellow(drive, limelight),
                    new NewAutoMoveOnReef(drive, limelight, right));
        } else {
            addCommands(
                    new AlignToAprilTagBlue(drive, limelight),
                    new AutoDriveToReefBlue(drive, limelight),
                    new AlignToAprilTagBlue(drive, limelight),
                    new NewAutoMoveOnReef(drive, limelight, !right));
        }

    }
}
