// package frc.robot.commands.intake;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.subsystems.Intake.IntakeConstants;
// import frc.robot.subsystems.Intake.IntakeSubsystem;

// public class EjectCoral extends Command {
//     private final IntakeSubsystem m_intake;

//     public EjectCoral(IntakeSubsystem intake) {
//         this.m_intake = intake;
//         addRequirements(intake);
//     }

//     @Override
//     public void initialize() {
//         m_intake.ejectRollers();
//     }

//     @Override
//     public boolean isFinished() {
//         // check if the light sensors are triggered
//         return !this.m_intake.getAnyLightSensor();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_intake.disableRollers();
//     }
// }

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class EjectHandoffCoral extends SequentialCommandGroup {
    public EjectHandoffCoral(
            IntakeSubsystem intakeSubsystem) {
        addCommands(
                new InstantCommand(() -> intakeSubsystem.ejectHandoffRollers()),
                new WaitUntilCommand(() -> intakeSubsystem.getAnyLightSensor()),
                new WaitCommand(0.5),
                new InstantCommand(() -> intakeSubsystem.disableRollers()));
    }
}
