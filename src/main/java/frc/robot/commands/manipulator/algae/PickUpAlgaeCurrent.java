package frc.robot.commands.manipulator.algae;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;

// if command is interrupted before algae is picked up, kill command
public class PickUpAlgaeCurrent extends Command {
        AlgaeSubsystem algaeSubsystem;
        Debouncer debouncer = new Debouncer(.5, Debouncer.DebounceType.kRising);
        private boolean algaePickedUp;

        public PickUpAlgaeCurrent(
                        AlgaeSubsystem algaeSubsystem) {

                this.algaeSubsystem = algaeSubsystem;

                addRequirements(algaeSubsystem);

        }

        @Override
        public void initialize() {
                System.out.println("Algae PickUp Starting");
                algaePickedUp = false;

        }

        @Override
        public void execute() {
                algaeSubsystem.pickup();
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        algaeSubsystem.stop();
                        System.out.println("Algae PickUp Interrupted");
                } else {
                        Commands.waitSeconds(.5);
                        algaeSubsystem.holdAlgae();
                        algaePickedUp = true;
                        System.out.println("Algae PickedUp");
                }
        }

        @Override
        public boolean isFinished() {
                return (debouncer
                                .calculate(algaeSubsystem.getCurrent() >= Constants.AlgaeConstants.kAlgaeCurrentLimit));
        }

}
