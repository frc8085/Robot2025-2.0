package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //up is negative acording to this motor
  private final TalonFX m_elevatorMotor = new TalonFX(23, "rio"); // change deviceID and canbus
  TalonFXConfiguration config = new TalonFXConfiguration();

  private double kSpeed = ElevatorConstants.kElevatorSpeed;

  public ElevatorSubsystem() {
    // Add configurations to Configs.java
    // https://pbs.twimg.com/media/F1Zwg4HacAEepQn.jpg:large
    //m_elevatorMotor.getConfigurator().apply(config);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;
    
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  public void periodic(){
    //SmartDashboard.getNumber("elePostion", m_elevatorMotor.getPosition());
    
  }

  public void moveUp() {
    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    m_elevatorMotor.setControl(m_request.withPosition(10));
  }

  // Maintain Position Inches
  public void keepPositionInches(double positionInches) {
    // set position in inches, convert to encoder value
    double position;
    position = positionInches * ElevatorConstants.kElevatorRevolutionsPerInch + 1;

    m_elevatorPIDController.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    m_elevatorMotor.set(0);
  }

  public void moveDown() {
    m_elevatorMotor.set(-kSpeed);
  }

  public void start() {
    m_elevatorMotor.set(kSpeed);
  }
}
