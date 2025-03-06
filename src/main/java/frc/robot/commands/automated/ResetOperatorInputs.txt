package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlgaeLevel;
import frc.robot.RobotContainer.CoralLevel;
import frc.robot.RobotContainer.ScoreDirection;

public class ResetOperatorInputs extends Command {
    @Override
    public void initialize() {
        RobotContainer.scoreDirection = ScoreDirection.UNDECIDED;
        RobotContainer.algaeLevel = AlgaeLevel.UNDECIDED;
        RobotContainer.coralLevel = CoralLevel.UNDECIDED;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}