package frc.robot.commands.windmill;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class LockPivotAndElevator extends Command {
    ElevatorSubsystem elevatorSubsystem;
    PivotSubsystem pivotSubsystem;
    IntakeSubsystem intakeSubsystem;

    public LockPivotAndElevator(ElevatorSubsystem elevatorSubsystem,
            PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(elevatorSubsystem, pivotSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        // SequentialCommandGroup commands = new SequentialCommandGroup();
        // commands.addCommands(new Windmill(elevatorSubsystem, pivotSubsystem,
        // Constants.Windmill.WindmillState.Home, false));
        elevatorSubsystem.setPos(ElevatorConstants.kElevatorStage1Height);
        pivotSubsystem.setPos(Rotation2d.fromDegrees(140));
        intakeSubsystem.setDeployRotation(Rotation2d.fromRotations(30));

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Elevator Unlocked");
    }

    @Override
    public boolean isFinished() {
        return false;

    }

}