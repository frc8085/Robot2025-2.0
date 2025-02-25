package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before algae is picked up, kill command
public class MoveAfterAlgaePickedUp extends Command {
        AlgaeSubsystem algaeSubsystem;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        public MoveAfterAlgaePickedUp(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {

                this.algaeSubsystem = algaeSubsystem;
                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
        }

        @Override
        public void initialize() {
                double currentPosition = elevatorSubsystem.getCurrentMotorPosition();
                elevatorSubsystem.setPos(currentPosition - 20);
                pivotSubsystem.setPos(Rotation2d.fromDegrees(0));
        }

        @Override
        public void execute() {
                
        }

        @Override
        public void end(boolean interrupted) {
                
                elevatorSubsystem.setPos(elevatorSubsystem.getCurrentMotorPosition());
                pivotSubsystem.setPos(pivotSubsystem.getCurrentRotation());
        }

        @Override
        public boolean isFinished() {
                return Math.abs(pivotSubsystem.getCurrentRotation().getDegrees())
                                - 0 < Constants.PivotArmConstants.kPivotTolerance.getDegrees();
        }
}
