package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoreDirection;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToScore extends Command {
    DriveSubsystem drive;

    public MoveToScore(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        switch (RobotContainer.scoreDirection) {
            case LEFT:
                // Move to the left
                break;
            case RIGHT:
                // Move to the right
                break;
            case UNDECIDED:
                // Do nothing until left or right is decided. (This can probably be empty)
                break;
        }
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}