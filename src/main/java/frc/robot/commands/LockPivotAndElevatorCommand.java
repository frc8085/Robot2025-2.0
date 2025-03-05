package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class LockPivotAndElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    PivotSubsystem pivotSubsystem;
    private boolean enableArm;

    public LockPivotAndElevatorCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(elevatorSubsystem, pivotSubsystem);
    }

    @Override
    public void initialize() {
        SequentialCommandGroup commands = new SequentialCommandGroup();

        enableArm = false;
        SmartDashboard.putBoolean("Enable Arm", enableArm);
        commands.addCommands(new Windmill(elevatorSubsystem, pivotSubsystem,
                Constants.Windmill.WindmillState.Home, false));
        elevatorSubsystem.setPos(Constants.ElevatorConstants.kElevatorStage1Height);
        pivotSubsystem.setPos(Rotation2d.fromDegrees(75));

    }

    @Override
    public void execute() {
        enableArm = SmartDashboard.getBoolean("Enable Arm", false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Elevator Unlocked");
    }

    @Override
    public boolean isFinished() {
        return (enableArm);
    }

    public void displayDisableSystem() {
        enableArm = SmartDashboard.getBoolean("Enable Arm", false);
    }
}