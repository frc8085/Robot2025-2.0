package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class NewAutoMoveOnReef extends SequentialCommandGroup {
    public NewAutoMoveOnReef(DriveSubsystem drive, LimelightSubsystem limelight, boolean climber) {
        if (climber) {
            addCommands(
                    // move in direction of climber
                    new ParallelRaceGroup(
                            new AutoDriveMeters(drive, -0.17, 0, 0.1),
                            new WaitCommand(2)),
                    new PrintCommand("Moved toward Climber"));
        } else {
            addCommands(
                    // move in direction away from climber
                    new ParallelRaceGroup(
                            new AutoDriveMeters(drive, 0.17, 0, 0.1),
                            new WaitCommand(2)),
                    new PrintCommand("Moved Away From Climber"));

        }
    }

}
