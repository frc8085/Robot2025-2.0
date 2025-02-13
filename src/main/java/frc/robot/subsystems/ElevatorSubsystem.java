package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(23, "rio"); // change deviceID and canbus
  TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoder m_elevatorEncoder = new CANcoder(23);

  private double kSpeed = ElevatorConstants.kElevatorSpeed;

  public ElevatorSubsystem() {
    // Add configurations to Configs.java
    // https://pbs.twimg.com/media/F1Zwg4HacAEepQn.jpg:large
    m_elevatorMotor.getConfigurator().apply(config);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;

    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  public void periodic() {
    SmartDashboard.getNumber("elePostion", kSpeed);
  }

  public void moveUp() {
    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    m_elevatorMotor.setControl(m_request.withPosition(2));

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
