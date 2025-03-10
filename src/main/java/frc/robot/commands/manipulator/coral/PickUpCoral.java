package frc.robot.commands.manipulator.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoral extends Command {
        CoralSubsystem coralSubsystem;

        private boolean coralPickedUp;

        public PickUpCoral(CoralSubsystem coralSubsystem) {

                this.coralSubsystem = coralSubsystem;

                addRequirements(coralSubsystem);

        }

        @Override
        public void initialize() {
                System.out.println("Coral PickUp Starting");
                coralPickedUp = false;
        }

        @Override
        public void execute() {
                if (coralSubsystem.isCoralDetected()) {
                        coralPickedUp = true;
                        coralSubsystem.stop();
                        System.out.println("Coral Picked Up");
                } else {
                        coralSubsystem.pickup();
                }
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        coralSubsystem.stop();
                        System.out.println("Coral PickUp Interrupted");
                } else {
                        System.out.println("Coral Picked Up");
                }
        }

        @Override
        public boolean isFinished() {
                return coralPickedUp;
        }

}
