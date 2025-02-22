package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToHome extends Command {
    ElevatorSubsystem elevator;

    public ElevatorToHome(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPos(WindmillState.Home.getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentMotorPosition()
                - WindmillState.Home.getElevatorHeight()) < Constants.ElevatorConstants.kElevatorTolerance;
    }
}