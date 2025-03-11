package frc.robot.commands.manipulator.coral;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoralCurrent extends Command {
        CoralSubsystem coralSubsystem;
        Debouncer debouncer = new Debouncer(.5, Debouncer.DebounceType.kRising);
        private boolean coralPickedUp;

        public PickUpCoralCurrent(CoralSubsystem coralSubsystem) {

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
                coralSubsystem.pickup();

        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        coralSubsystem.stop();
                        System.out.println("Coral PickUp Interrupted");
                } else {
                        Commands.waitSeconds(.25);
                        coralSubsystem.stop();
                        System.out.println("Coral Picked Up");
                }
        }

        @Override
        public boolean isFinished() {
                return (debouncer
                                .calculate(coralSubsystem.getCurrent() >= Constants.CoralConstants.kCoralCurrentLimit));
        }

}
