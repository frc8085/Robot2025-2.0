package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoral extends Command {
        CoralSubsystem coralSubsystem;

        private boolean coralPickedUp = false;

        public PickUpCoral(
                        CoralSubsystem coralSubsystem) {

                this.coralSubsystem = coralSubsystem;

                addRequirements(coralSubsystem);

        }

        @Override
        public void initialize() {
                coralPickedUp = false;
        }

        @Override
        public void execute() {
                if (coralSubsystem.isCoralDetected()) {
                        coralPickedUp = true;
                        coralSubsystem.stop();
                } else {
                        coralSubsystem.pickup();
                }
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        System.out.println("Coral PickUp Interrupted");
                        coralSubsystem.stop();
                } else {
                        System.out.println("Coral Picked Up");
                }
                coralPickedUp = false;
        }

        @Override
        public boolean isFinished() {
                return coralPickedUp;
        }

}
