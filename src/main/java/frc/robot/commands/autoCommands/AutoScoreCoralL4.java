// package frc.robot.commands.autoCommands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.commands.manipulator.coral.EjectCoral;
// import frc.robot.commands.states.ToCoralDropOff4;
// import frc.robot.subsystems.AlgaeSubsystem;
// import frc.robot.subsystems.CoralSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.PivotSubsystem;

// public class AutoScoreCoralL4 extends SequentialCommandGroup {
// public AutoScoreCoralL4(
// AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
// PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, DriveSubsystem
// driveSubsystem,
// boolean yellow) {
// addCommands(
// new RunCommand(() -> driveSubsystem.lock()),
// new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, yellow),
// new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff4Height),
// new WaitUntilCommand(() -> pivotSubsystem.pivotAtCoral4DropOffAngle(yellow)),
// new WaitCommand(.5),
// new PrintCommand("Coral Eject Started"),
// new RunCommand(() -> coralSubsystem.eject(),
// coralSubsystem).withTimeout(0.5),
// new InstantCommand(coralSubsystem::stop));
// }
// }
