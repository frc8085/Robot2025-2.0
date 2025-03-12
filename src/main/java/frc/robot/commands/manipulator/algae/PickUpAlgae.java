package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before algae is picked up, kill command
public class PickUpAlgae extends Command {
        AlgaeSubsystem algaeSubsystem;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        public PickUpAlgae(
                        AlgaeSubsystem algaeSubsystem) {

                this.algaeSubsystem = algaeSubsystem;

                addRequirements(algaeSubsystem);

        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
                algaeSubsystem.pickup();
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        algaeSubsystem.stop();
                } else {
                        Commands.waitSeconds(.25);
                        algaeSubsystem.holdAlgae();
                }
        }

        @Override
        public boolean isFinished() {
                return algaeSubsystem.isAlgaeDetected();
        }

}
