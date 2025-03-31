// package frc.robot.commands.windmill;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.commands.windmill.elevator.*;
// import frc.robot.commands.windmill.pivot.*;

// public class InitializePivotAndElevator extends
// SequentialCommandGroup {

// public InitializePivotAndElevator(PivotSubsystem pivotSubsystem,
// ElevatorSubsystem elevatorSubsystem) {
// addCommands(
// new ParallelCommandGroup(
// new InitializePivot(pivotSubsystem),
// new ZeroElevator(elevatorSubsystem)));
// }
// }