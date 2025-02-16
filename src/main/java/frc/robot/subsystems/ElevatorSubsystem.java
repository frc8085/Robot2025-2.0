package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

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

  public ElevatorSubsystem() {

    // Defining Elevator PIDs
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;
    slot0Configs.kV = ElevatorConstants.kElevatorV;
    slot0Configs.kA = ElevatorConstants.kElevatorA;

    // Setting Motor Configs
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Setting Motion Magic Configs
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorMMVelo;
    config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kElevatorMMAcc;
    config.MotionMagic.MotionMagicJerk = Constants.ElevatorConstants.kElevatorMMJerk;

    // get sensor readings
    elevatorPosition = m_elevatorMotor.getPosition();

    // Apply Configs
    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  // Setting the height of the elevator
  public void setPos(double inches) {

    // Checks to see if the elevator height is below the minimum, and if it is, set
    // it to the minimum and
    // check to see if the elevator height is above the maximum, and if it is, set
    // it to the maximum

    if (inches < Constants.ElevatorConstants.kElevatorMin) {
      inches = Constants.ElevatorConstants.kElevatorMin;
    } else if (inches > Constants.ElevatorConstants.kElevatorMax) {
      inches = Constants.ElevatorConstants.kElevatorMax;
    }

    // set the feedforward value based on the elevator height
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

  // set the zero value of the motor encoder
  public void zero(double inches) {
    m_elevatorMotor.setPosition(
        inches * Constants.ElevatorConstants.kElevatorMotorGearRatio);
  }

  // read the current elevator encoder position
  public double getCurrentMotorPosition() {
    elevatorPosition.refresh();
    return elevatorPosition.getValueAsDouble();
  }

  // translate the current elevator encoder position into elevator height
  public double getCurrentHeight() {
    return (getCurrentMotorPosition() / Constants.ElevatorConstants.kElevatorMotorGearRatio);
  }

  public void periodic() {
    // display encoder readings on dashboard
    SmartDashboard.putNumber("current Motor Position", getCurrentMotorPosition());
    SmartDashboard.putNumber("current Height", getCurrentHeight());

  }

  // turn off elevator
  public void stop() {
    m_elevatorMotor.set(0);
  }

  // open loop move elevator up & down
  public void moveDown() {
    m_elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  public void moveUp() {
    m_elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  public void holdHeight() {
    double targetPosition = getCurrentHeight();
    setPos(targetPosition);
  }
}
