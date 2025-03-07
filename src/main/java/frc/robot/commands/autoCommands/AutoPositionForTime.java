package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoPositionForTime extends SequentialCommandGroup {
    public AutoPositionForTime(DriveSubsystem drive, LimelightSubsystem limelight, boolean yellow, boolean right,
            double time) {
        if (right) {
            if (yellow) {
                addCommands(
                        new ParallelDeadlineGroup(
                                new WaitCommand(time),
                                new AutoDriveMeters(drive, 0, -1, 0.2))); // move toward yellow;
            } else {
                addCommands(
                        new ParallelDeadlineGroup(
                                new WaitCommand(time),
                                new AutoDriveMeters(drive, 0, 1, 0.2))); // move toward blue;

            }
        } else {
            if (!yellow) {
                addCommands(
                        new ParallelDeadlineGroup(
                                new WaitCommand(time),
                                new AutoDriveMeters(drive, 0, -1, 0.2))); // move away from blue;
            } else {
                addCommands(
                        new ParallelDeadlineGroup(
                                new WaitCommand(time),
                                new AutoDriveMeters(drive, 0, 1, 0.2))); // move away from yellow;

            }
        }
    }
}