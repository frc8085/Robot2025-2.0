package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;

public class Windmill extends SequentialCommandGroup {

        private ParallelCommandGroup ParallelWindmill(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem,
                        double targetHeight, Rotation2d targetAngle) {
                return new ParallelCommandGroup(
                                new PrintCommand("Windmill: Elevator not in danger zone"),
                                new PrintCommand("Elevator motor position: "
                                                + elevatorSubsystem.getCurrentMotorPosition()),
                                new Elevator(elevatorSubsystem, targetHeight),
                                new Pivot(pivotSubsystem, targetAngle));
        }

        private SequentialCommandGroup ElevatorDangerZone(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, double targetHeight, Rotation2d targetAngle) {
                return new SequentialCommandGroup(
                                new PrintCommand("Windmill: Elevator in danger zone"),
                                new PrintCommand("Elevator motor position: "
                                                + elevatorSubsystem.getCurrentMotorPosition()),
                                new Elevator(elevatorSubsystem,
                                                Constants.ElevatorConstants.kElevatorSafeHeightMax),
                                new Pivot(pivotSubsystem, targetAngle),
                                new Elevator(elevatorSubsystem, targetHeight));
        }

        private void constructWindmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        double targetHeight,
                        Rotation2d targetAngle) {
                addCommands(new ConditionalCommand(
                                // On true, if elevator is not in danger zone, run elevator and pivot in
                                // parallel
                                ParallelWindmill(elevatorSubsystem, pivotSubsystem, targetHeight,
                                                targetAngle),
                                // On false, if elevator is in danger zone, run elevator and pivot in
                                ElevatorDangerZone(elevatorSubsystem, pivotSubsystem, targetHeight,
                                                targetAngle),
                                () -> !elevatorSubsystem.inDangerZone()));
        }

        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
                        Rotation2d targetAngle) {
                constructWindmill(elevatorSubsystem, pivotSubsystem, targetHeight, targetAngle);
        }

        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState windmillState,
                        boolean mirrored) {
                var rotation_target = windmillState.getPivotArmAngle();
                if (mirrored && windmillState.canMirror()) {
                        // mirror the windmill pivot arm if possible (flip around the -90 degree angle)
                        Rotation2d rotation_diff = Rotation2d.fromDegrees(-90).minus(rotation_target);
                        rotation_target = Rotation2d.fromDegrees(-90).plus(rotation_diff);

                }
                constructWindmill(elevatorSubsystem, pivotSubsystem, windmillState.getElevatorHeight(),
                                rotation_target);
        }

}