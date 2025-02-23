package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import java.util.Vector;

public class Windmill extends Command {

        double targetHeight;
        Rotation2d targetAngle;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        boolean finished = false;

        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
                        Rotation2d targetAngle) {

                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = targetAngle;
                this.targetHeight = targetHeight;
        }

        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState windmillState,
                        boolean mirrored) {
                var rotation_target = windmillState.getPivotArmAngle();
                if (mirrored && windmillState.canMirror()) {
                        // mirror the windmill pivot arm if possible (flip around the -90 degree angle)
                        rotation_target = Rotation2d.fromDegrees(-rotation_target.getDegrees());
                }
                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = rotation_target;
                this.targetHeight = windmillState.getElevatorHeight();
        }

        // public void finish() {
        // this.finished = true;
        // }

        @Override
        public void initialize() {
                boolean elevatorInDangerZone = elevatorSubsystem.inDangerZone();
                boolean elevatorEndInDangerZone = elevatorSubsystem.targetInDangerZone(this.targetHeight);
                boolean pivotWillSwingThrough = pivotSubsystem.willPivotThroughDangerZone(this.targetAngle);

                SequentialCommandGroup commands = new SequentialCommandGroup();

                if (elevatorInDangerZone && pivotWillSwingThrough) {
                        commands.addCommands(new PrintCommand("Windmill: Pivot will swing through danger zone"));
                } else {
                        commands.addCommands(new PrintCommand("Windmill: Pivot will not swing through danger zone"));
                }

                if (!elevatorInDangerZone || !pivotWillSwingThrough) {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new Elevator(elevatorSubsystem, targetHeight),
                                                        new Pivot(pivotSubsystem, targetAngle)));

                } else {
                        if (elevatorEndInDangerZone) {
                                commands.addCommands(
                                                new SequentialCommandGroup(
                                                                new Elevator(elevatorSubsystem,
                                                                                Constants.ElevatorConstants.kElevatorSafeHeightMax),
                                                                new Pivot(pivotSubsystem, targetAngle),
                                                                new Elevator(elevatorSubsystem, targetHeight)));
                        } else {
                                commands.addCommands(
                                                new ParallelCommandGroup(
                                                                new Elevator(elevatorSubsystem, targetHeight),
                                                                new SequentialCommandGroup(
                                                                                new WaitUntilCommand(
                                                                                                () -> !elevatorSubsystem
                                                                                                                .inDangerZone()),
                                                                                new Pivot(pivotSubsystem,
                                                                                                targetAngle))));

                        }
                }

                CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                                commands
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