package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class ScoreAlgae extends Command {
        AlgaeSubsystem algaeSubsystem;

        public ScoreAlgae(
                        AlgaeSubsystem algaeSubsystem) {

                this.algaeSubsystem = algaeSubsystem;
        }

        public void initialize() {
        }

        @Override
        public void execute() {
                algaeSubsystem.eject();
        }

        @Override
        public void end(boolean interrupted) {
                algaeSubsystem.stop();
        }

        @Override
        public boolean isFinished() {
                new WaitCommand(.5);
                return true;
        }
}
