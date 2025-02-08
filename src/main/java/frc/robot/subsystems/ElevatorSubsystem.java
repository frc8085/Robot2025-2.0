package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(24, "rio"); // change deviceID and canbus
  TalonFXConfiguration config = new TalonFXConfiguration();

  private double kSpeed = ElevatorConstants.kElevatorSpeed;

  public ElevatorSubsystem() {
    // Add configurations to Configs.java
    m_elevatorMotor.getConfigurator().apply(config);
    config.Slot0.kP = ElevatorConstants.kElevatorP;
    config.Slot0.kI = ElevatorConstants.kElevatorI;
    config.Slot0.kD = ElevatorConstants.kElevatorD;

  }

  public void moveUp() {
    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    m_elevatorMotor.setControl(m_request.withPosition(10));
  }

  public void stop() {
    m_elevatorMotor.set(0);
  }

  public void moveDown() {
    m_elevatorMotor.set(-kSpeed);
  }
}
