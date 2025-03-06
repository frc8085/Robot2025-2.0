package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AutoMoveForwardForTime;
import frc.robot.commands.scoring.ScoreCoralL3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class MoveAndScoreCoralL3 extends SequentialCommandGroup {
        public MoveAndScoreCoralL3(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
                        LimelightSubsystem limelight,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
                        boolean yellow) {
                addCommands(
                                new AutoMoveForwardForTime(driveSubsystem, limelight, yellow,
                                                0.5),
                                new ScoreCoralL3(elevatorSubsystem, pivotSubsystem, coralSubsystem, yellow));

        }
}