package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpAlgaeL3 extends Command {
        AlgaeSubsystem algaeSubsystem;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        public PickUpAlgaeL3(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {

                this.algaeSubsystem = algaeSubsystem;
                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
                elevatorSubsystem.setPos(ElevatorConstants.kElevatorReef3Height - 20);
                pivotSubsystem.setPos(Rotation2d.fromDegrees(-40));
        }

        @Override
        public void end(boolean interrupted) {
                elevatorSubsystem.setPos(elevatorSubsystem.getCurrentMotorPosition());
                pivotSubsystem.setPos(pivotSubsystem.getCurrentRotation());
        }

        @Override
        public boolean isFinished() {
                return Math.abs(elevatorSubsystem.getCurrentMotorPosition()
                                - (WindmillState.AlgaePickUpReef3.getElevatorHeight()
                                                - 20)) < Constants.ElevatorConstants.kElevatorTolerance;
        }
}
