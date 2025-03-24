package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;

public class ChoreoAutoSimple extends SequentialCommandGroup {
    public ChoreoAutoSimple(DriveSubsystem driveSubsystem) {

        addCommands(
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("SIMPLEAUTO1"), true));
    }
}