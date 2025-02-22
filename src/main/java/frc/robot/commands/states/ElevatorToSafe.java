package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;

// Puts the elevator to a safe position if necessary
public class ElevatorToSafe extends Command {
    ElevatorSubsystem elevatorSubsystem;
    WindmillState prevState;

    public ElevatorToSafe(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (elevatorSubsystem.getCurrentMotorPosition() < Constants.ElevatorConstants.kElevatorSafeHeightMax) {
            // Put the elevator to safe height first
            elevatorSubsystem.setPos(Constants.ElevatorConstants.kElevatorSafeHeightMax);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish the command if either we came from a safe state or we finished
        // reaching a safe state
        return (elevatorSubsystem.getCurrentMotorPosition() > Constants.ElevatorConstants.kElevatorSafeHeightMax)
                || (Math.abs(elevatorSubsystem.getCurrentMotorPosition()
                        - Constants.ElevatorConstants.kElevatorSafeHeightMax) < Constants.ElevatorConstants.kElevatorTolerance);
        // Checks if the difference b/w the current and target position is less than a
        // threshold
    }
}