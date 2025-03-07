package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AutoMoveForwardForTime;
import frc.robot.commands.scoring.ScoreCoralL1;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class MoveAndScoreCoralL1 extends SequentialCommandGroup {
        public MoveAndScoreCoralL1(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
                        LimelightSubsystem limelight,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
                        boolean yellow) {
                addCommands(
                                new AutoMoveForwardForTime(driveSubsystem, limelight, yellow,
                                                2),
                                new ScoreCoralL1(elevatorSubsystem, pivotSubsystem, coralSubsystem, yellow));

        }
}