package frc.robot.commands.windmill;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class UnlockClimber extends Command {
    IntakeSubsystem intakeSubsystem;

    public UnlockClimber(IntakeSubsystem intakeSubsystem) {

        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setDeployRotation(IntakeConstants.kIntakeOutAngle);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}