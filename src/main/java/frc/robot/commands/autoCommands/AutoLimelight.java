package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagBlue;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.NewAutoMoveOnReef;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelight extends SequentialCommandGroup {
    public AutoLimelight(DriveSubsystem drive, LimelightSubsystem limelight, boolean right, boolean yellow) {
        if (yellow) {
            addCommands(
                    new AlignToAprilTagYellow(drive, limelight),
                    new PrintCommand("Angle to Reef"),
                    new AutoDriveToReefYellow(drive, limelight),
                    new PrintCommand("Align to Reef"));
        } else {
            addCommands(
                    new AlignToAprilTagBlue(drive, limelight),
                    new PrintCommand("Angle to Reef"),
                    new AutoDriveToReefBlue(drive, limelight),
                    new PrintCommand("Align to Reef"));
        }

    }
}
