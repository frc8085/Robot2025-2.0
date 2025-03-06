package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AutoPosition;
import frc.robot.commands.scoring.AlignToAprilTagYellow;
import frc.robot.commands.scoring.DriveToReefYellow;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightPosition extends SequentialCommandGroup {
    public AutoLimelightPosition(DriveSubsystem drive, LimelightSubsystem limelight, boolean right) {
        addCommands(
                new DriveToReefYellow(drive, limelight),
                new AlignToAprilTagYellow(drive, limelight),
                new AutoPosition(drive, limelight, right));
    }

}
