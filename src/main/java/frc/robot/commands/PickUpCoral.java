package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoral extends Command {
        CoralSubsystem coralSubsystem;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        public PickUpCoral(
                        CoralSubsystem coralSubsystem) {

                this.coralSubsystem = coralSubsystem;

                addRequirements(coralSubsystem);

        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
                coralSubsystem.pickup();
        }

        @Override
        public void end(boolean interrupted) {
                coralSubsystem.stop();
        }

        @Override
        public boolean isFinished() {
                return coralSubsystem.isCoralDetected();
        }

}
