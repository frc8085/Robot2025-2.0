// package frc.robot.commands.intake;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.Constants.IntakeConstants;

// public class DeployIntake extends Command {
// private final Rotation2d m_targetAngle = IntakeConstants.kIntakeInAngle;
// private final IntakeSubsystem m_intake;
// private final Rotation2d tolerance = IntakeConstants.kIntakeTolerance;

// public DeployIntake(IntakeSubsystem intake) {
// this.m_intake = intake;
// addRequirements(intake);
// }

// // @Override
// public void initialize() {
// m_intake.setDeployRotation(m_targetAngle);
// m_intake.enableRollers();
// }

// @Override
// public boolean isFinished() {
// // check if the light sensors are triggered
// return this.m_intake.getAnyLightSensor();
// }

// @Override
// public void end(boolean interrupted) {
// // m_intake.setDeployRotation(IntakeConstants.kIntakeInAngle);
// m_intake.disableRollers();
// }
// }