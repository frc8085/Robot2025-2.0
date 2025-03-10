package frc.robot.commands.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractClimb extends Command {
    ClimberSubsystem climberSubsystem;

    public RetractClimb(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);

    }

    @Override
    public void initialize() {
        climberSubsystem.moveDown();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.climberAtHomePosition();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
