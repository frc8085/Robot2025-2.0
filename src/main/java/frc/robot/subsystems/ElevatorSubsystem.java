package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIdConstants.kElevatorCanId, "rio"); // device id and
                                                                                                       // canbus
  TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoder m_elevatorEncoder = new CANcoder(Constants.CanIdConstants.kElevatorCancoderCanID);
  private StatusSignal<Angle> elevatorPosition;

  private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

  private double kSpeed = ElevatorConstants.kElevatorSpeed;

  public ElevatorSubsystem() {

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;
    slot0Configs.kV = ElevatorConstants.kElevatorV;
    slot0Configs.kA = ElevatorConstants.kElevatorA;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorMMVelo;
    config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kElevatorMMAcc;
    config.MotionMagic.MotionMagicJerk = Constants.ElevatorConstants.kElevatorMMJerk;

    elevatorPosition = m_elevatorMotor.getPosition();

    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  public void setPos(double inches) {

    if (inches < Constants.ElevatorConstants.kElevatorMin) {
      inches = Constants.ElevatorConstants.kElevatorMin;
    } else if (inches > Constants.ElevatorConstants.kElevatorMax) {
      inches = Constants.ElevatorConstants.kElevatorMax;
    }

    double ff = 0;
    if (inches > ElevatorConstants.kElevatorStage2Height) {
      ff = ElevatorConstants.kElevatorStage3FF;
    } else if (inches > Constants.ElevatorConstants.kElevatorStage1Height) {
      ff = ElevatorConstants.kElevatorStage2FF;
    }

    motionMagicControl.Position = inches * Constants.ElevatorConstants.kElevatorMotorGearRatio;
    motionMagicControl.FeedForward = ff;
    m_elevatorMotor.setControl(motionMagicControl);
    SmartDashboard.putNumber("height value", inches);
  }

  public void zero(double inches) {
    m_elevatorMotor.setPosition(
        inches * Constants.ElevatorConstants.kElevatorMotorGearRatio);
  }

  public double getCurrentMotorPosition() {
    elevatorPosition.refresh();
    return elevatorPosition.getValueAsDouble();
  }

  public double getCurrentHeight() {
    return (getCurrentMotorPosition() / Constants.ElevatorConstants.kElevatorMotorGearRatio);
  }

  public void periodic() {
    // Get motor readings
    SmartDashboard.putNumber("current Motor Position", getCurrentMotorPosition());
    SmartDashboard.putNumber("current Height", getCurrentHeight());

  }

  public void stop() {
    m_elevatorMotor.set(0);
  }

  /*
   * Open Loop Stuff
   * public void moveDown() {
   * m_elevatorMotor.set(-kSpeed);
   * }
   * 
   * public void start() {
   * m_elevatorMotor.set(kSpeed);
   * }
   */
}
