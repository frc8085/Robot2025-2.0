package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToAlgaeNetRight extends Command {
    ElevatorSubsystem elevator;

    public ElevatorToAlgaeNetRight(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPos(WindmillState.AlgaeNetRight.getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentMotorPosition()
                - WindmillState.AlgaeNetRight.getElevatorHeight()) < Constants.ElevatorConstants.kElevatorTolerance;
    }
}