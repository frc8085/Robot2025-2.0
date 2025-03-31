package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

public class AlgaeSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = TuningModeConstants.kAlgaeTuning;

  // import motor id
  private final SparkFlex m_algaeMotor = new SparkFlex(AlgaeConstants.kAlgaeCanId,
      MotorDefaultsConstants.NeoVortexMotorType);
  SparkFlexConfig config = new SparkFlexConfig();
  private RelativeEncoder m_algaeEncoder;
  private SparkClosedLoopController m_algaePIDController;

  // light sensor
  DigitalInput lightSensor = new DigitalInput(AlgaeConstants.kIRPort);

  // robot does not start with algae
  private boolean algaeTrue = false;

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
    m_algaeMotor.configure(AlgaeConfig.algaeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public double getCurrent() {
    return m_algaeMotor.getOutputCurrent();
  }

  public void pickup() {
    m_algaeEncoder.setPosition(0);
    m_algaeMotor.set(-AlgaeConstants.kAlgaeIntakeSpeed);
  }

  public void stop() {
    m_algaeMotor.set(0);
  }

  public void eject() {
    m_algaeMotor.set(AlgaeConstants.kAlgaeEjectSpeed);
  }

  /** Resets the Intake encoder to currently read a position of 0. */
  public void reset() {
    m_algaeEncoder.setPosition(0);
  }

  public void holdAlgae() {
    double targetPosition = CurrentAlgaeEncoderPosition();
    m_algaePIDController.setReference(targetPosition, ControlType.kPosition);

  }

  public Boolean isAlgaeDetected() {
    return lightSensor.get();
  }

  /* When a algae is picked up, it's in the robot */
  public void algaeStatus() {
    if (lightSensor.get()) {
      algaeTrue = true;
    } else {
      algaeTrue = false;
    }
  }

  /* Give us a state when the note is in robot */
  public boolean algaeInRobot() {
    return algaeTrue;
  }

  @Override
  public void periodic() {
    if (TUNING_MODE) {
    }
  }

}
