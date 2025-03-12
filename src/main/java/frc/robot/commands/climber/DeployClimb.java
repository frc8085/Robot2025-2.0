package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;

// if command is interrupted before algae is picked up, kill command
public class DeployClimb extends Command {
    ClimberSubsystem climberSubsystem;

    public DeployClimb(
            ClimberSubsystem climberSubsystem) {

        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);

    }

    @Override
    public void initialize() {
        climberSubsystem.zeroAtLimitSwitch();

    }

    @Override
    public void execute() {
        climberSubsystem.deploy();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.climberDeployed();
    }

}
