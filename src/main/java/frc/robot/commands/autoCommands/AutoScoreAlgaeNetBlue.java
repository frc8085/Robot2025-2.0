// package frc.robot.commands.autoCommands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.commands.manipulator.algae.EjectAlgae;
// import frc.robot.commands.states.ToAlgaeNet;
// import frc.robot.commands.states.ToHomeCommand;
// import frc.robot.subsystems.AlgaeSubsystem;
// import frc.robot.subsystems.CoralSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.PivotSubsystem;

// public class AutoScoreAlgaeNetBlue extends SequentialCommandGroup {
// public AutoScoreAlgaeNetBlue(
// AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
// PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem) {
// addCommands(
// new ToAlgaeNet(elevatorSubsystem, pivotSubsystem, false),
// new WaitUntilCommand(elevatorSubsystem::elevatorAtAlgaeScoreHeight),
// new EjectAlgae(algaeSubsystem),
// new WaitCommand(0.25),
// new ToHomeCommand(elevatorSubsystem, pivotSubsystem));

// }
// }
