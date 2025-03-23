package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.manipulator.coral.EjectCoral;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;

public class AutoSimple extends SequentialCommandGroup {
    public AutoSimple(DriveSubsystem driveSubsystem) {

        addCommands(
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("SIMPLEAUTO1"), true));
    }
}