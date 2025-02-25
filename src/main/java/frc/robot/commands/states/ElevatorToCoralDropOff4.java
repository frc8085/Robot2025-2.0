package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToCoralDropOff4 extends Command {
    ElevatorSubsystem elevator;

    public ElevatorToCoralDropOff4(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPos(WindmillState.CoralDropOff4.getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentMotorPosition()
                - WindmillState.CoralDropOff4.getElevatorHeight()) < Constants.ElevatorConstants.kElevatorTolerance;
    }
}