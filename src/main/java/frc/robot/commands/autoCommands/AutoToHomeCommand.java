// package frc.robot.commands.autoCommands;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.Constants;
// import frc.robot.commands.windmill.Windmill;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;

// public class AutoToHomeCommand extends ParallelCommandGroup {
// public AutoToHomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem
// pivotSubsystem) {
// addCommands(
// new PrintCommand("Auto Move to Home"),
// new Windmill(elevatorSubsystem, pivotSubsystem,
// Constants.Windmill.WindmillState.Home, false));
// }
// }