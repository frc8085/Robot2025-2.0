package frc.robot.commands.windmill;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.commands.windmill.pivot.*;
import frc.robot.commands.windmill.elevator.*;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.Constants.Windmill.WindmillState;

public class WindmillAlgaeNet extends Command {

        double targetHeight;
        Rotation2d targetAngle;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        boolean finished = false;

        public WindmillAlgaeNet(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
                        Rotation2d targetAngle) {

                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = targetAngle;
                this.targetHeight = targetHeight;
        }

        @Override
        public void initialize() {

                SequentialCommandGroup commands = new SequentialCommandGroup();

                commands.addCommands(
                                new Pivot(pivotSubsystem, targetAngle),
                                new WaitUntilCommand(() -> pivotSubsystem.inDangerZone()),
                                new Elevator(elevatorSubsystem,
                                                Constants.ElevatorConstants.kElevatorSafeHeightMax),
                                new Elevator(elevatorSubsystem, targetHeight));

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(commands
                // new InstantCommand(() -> this.finish())
                ));

        }

        @Override
        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        this.finished = true;
                }
        }
}