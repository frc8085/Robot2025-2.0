package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = TuningModeConstants.kAlgaeTuning;

  // import motor id
  private final SparkMax m_algaeMotor = new SparkMax(CanIdConstants.kAlgaeCanId,
      MotorDefaultsConstants.Neo550MotorType);
  SparkMaxConfig config = new SparkMaxConfig();
  private RelativeEncoder m_algaeEncoder;
  private SparkClosedLoopController m_algaePIDController;

  // Determine current intake encoder position
  public double CurrentAlgaeEncoderPosition() {
    return m_algaeEncoder.getPosition();
  }

  public AlgaeSubsystem() {

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_algaeEncoder = m_algaeMotor.getEncoder();
    m_algaePIDController = m_algaeMotor.getClosedLoopController();
    m_algaeMotor.configure(Configs.AlgaeManipulator.algaeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public void pickup() {
    m_algaeEncoder.setPosition(0);
    m_algaeMotor.set(-AlgaeConstants.kAlgaeSpeed);
  }

  public void stop() {
    m_algaeMotor.set(0);
  }

  public void eject() {
    m_algaeMotor.set(AlgaeConstants.kAlgaeSpeed);
  }

  /** Resets the Intake encoder to currently read a position of 0. */
  public void reset() {
    m_algaeEncoder.setPosition(0);
  }

  public void holdAlgae() {
    double targetPosition = CurrentAlgaeEncoderPosition();
    m_algaePIDController.setReference(targetPosition, ControlType.kPosition);

  }

  @Override
  public void periodic() {
    if (TUNING_MODE) {
    }
  }

}
