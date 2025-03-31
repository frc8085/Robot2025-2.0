// package frc.robot.commands.states;

// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.commands.windmill.Windmill;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.PivotSubsystem;

// public class ToCoralSource extends SequentialCommandGroup {
// public ToCoralSource(ElevatorSubsystem elevatorSubsystem, PivotSubsystem
// pivotSubsystem, boolean altButton) {
// if (altButton) {
// addCommands(
// new PrintCommand("Move to Coral Pick Up Alt"),
// new Windmill(elevatorSubsystem, pivotSubsystem,
// Constants.Windmill.WindmillState.CoralPickupAltHigher,
// false));
// } else {
// addCommands(
// new PrintCommand("Move to Coral Pick Up"),
// new Windmill(elevatorSubsystem, pivotSubsystem,
// Constants.Windmill.WindmillState.CoralPickupHigher,
// false));

// }
// }

// }
