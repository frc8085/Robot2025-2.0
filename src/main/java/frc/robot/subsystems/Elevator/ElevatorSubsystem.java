package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.States.DriveState;

public class ElevatorSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = TuningModeConstants.kElevatorTuning;

  // CAN ID and CANbus
  private final TalonFX m_elevatorMotor = new TalonFX(ElevatorConstants.kElevatorCanId, "rio");
  TalonFXConfiguration m_elevatorMotorConfig = new TalonFXConfiguration();
  private final CANcoder m_elevatorEncoder = new CANcoder(ElevatorConstants.kElevatorCancoderCanId);

  // get encoder data
  private StatusSignal<Angle> elevatorPosition;
  private StatusSignal<AngularVelocity> elevatorVelocity;

  private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

  private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);

  private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  // Limit Switches
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(2);
  DigitalInput zeroLimitSwitch = new DigitalInput(1);

  final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);

  public boolean ElevatorLowerLimitHit() {
    return bottomLimitSwitch.get();
  }

  public boolean ElevatorRaiseLimitHit() {
    return topLimitSwitch.get();
  }

  public boolean ElevatorZeroLimitHit() {
    return zeroLimitSwitch.get();
  }

  public boolean eitherLimitSwitchPressed() {
    return topLimitSwitch.get() || bottomLimitSwitch.get();
  }

  public ElevatorSubsystem() {

    // Defining Elevator PIDs
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;
    slot0Configs.kV = ElevatorConstants.kElevatorV;
    slot0Configs.kA = ElevatorConstants.kElevatorA;
    slot0Configs.kG = ElevatorConstants.kElevatorG;
    slot0Configs.kS = ElevatorConstants.kElevatorS;

    // Setting Motor Configs
    m_elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Setting Motion Magic Configs
    m_elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMMVelo;
    m_elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMMAcc;
    m_elevatorMotorConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorMMJerk;

    /* Current Limiting for the elevator */
    m_currentLimits.SupplyCurrentLimit = 30; // Limit to 30 amps
    m_currentLimits.SupplyCurrentLowerLimit = 50; // If we exceed 50 amps
    m_currentLimits.SupplyCurrentLowerTime = 0.1; // For at least 0.1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 60; // Limit stator to 60 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
    m_elevatorMotorConfig.CurrentLimits = m_currentLimits;

    // get sensor readings
    elevatorPosition = m_elevatorMotor.getPosition();
    elevatorVelocity = m_elevatorEncoder.getVelocity();

    // Apply Configs
    m_elevatorMotor.getConfigurator().apply(m_elevatorMotorConfig);
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  // Setting the height of the elevator
  public void setPos(double rotations) {

    // Checks to see if the elevator height is below the minimum, and if it is, set
    // it to the minimum and
    // check to see if the elevator height is above the maximum, and if it is, set
    // it to the maximum

    if ((rotations < ElevatorConstants.kElevatorMin)) {
      rotations = ElevatorConstants.kElevatorMin;
    } else if ((rotations > ElevatorConstants.kElevatorMax)) {
      rotations = ElevatorConstants.kElevatorMax;
    }

    // set the feedforward value based on the elevator height
    double ff = 0;
    if (rotations > ElevatorConstants.kElevatorStage2Height) {
      ff = ElevatorConstants.kElevatorStage3FF;
    } else if (rotations > ElevatorConstants.kElevatorStage1Height) {
      ff = ElevatorConstants.kElevatorStage2FF;
    }

    motionMagicPositionControl.Position = rotations;
    motionMagicVelocityControl.FeedForward = ff;
    m_elevatorMotor.setControl(motionMagicPositionControl);
  }

  public void zero() {
    m_elevatorMotor.setPosition(
        ElevatorConstants.kElevatorStage1Height);
  }

  // read the current elevator encoder position
  public double getCurrentMotorPosition() {
    elevatorPosition.refresh();
    return elevatorPosition.getValueAsDouble();
  }

  // read the current elevator encoder velocity
  public double getCurrentMotorVelocity() {
    elevatorVelocity.refresh();
    return elevatorVelocity.getValueAsDouble();
  }

  public boolean targetInDangerZone(double target_position) {
    return target_position < (ElevatorConstants.kElevatorSafeHeightMax);
  }

  public boolean isSafeToPivot() {
    return getCurrentMotorPosition() >= 50;
  }

  public boolean elevatorAtAlgaeScoreHeight() {
    return getCurrentMotorPosition() >= (ElevatorConstants.kElevatorNetHeight
        - ElevatorConstants.kElevatorTolerance);
  }

  public boolean elevatorBelowSafeTravelHeight() {
    return (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorSafeTravelHeight
        + ElevatorConstants.kElevatorTolerance));
  }

  public boolean elevatorAtCoralDropOff1Height() {
    return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorCoralDropOff1Height
        - ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorCoralDropOff1Height
            + ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff2Height() {
    return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorCoralDropOff2Height
        - ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorCoralDropOff2Height
            + ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff3Height() {
    return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorCoralDropOff3Height
        - ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorCoralDropOff3Height
            + ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff4Height() {
    return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorCoralDropOff4Height
        - ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorCoralDropOff4Height
            + ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtAlgaeReefL2(boolean yellow) {
    if (!yellow) {
      return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorReef2Height
          - ElevatorConstants.kElevatorTolerance))
          && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorReef2Height
              + ElevatorConstants.kElevatorTolerance)));
    } else {
      return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorReef2FlipHeight
          - ElevatorConstants.kElevatorTolerance))
          && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorReef2FlipHeight
              + ElevatorConstants.kElevatorTolerance)));
    }
  }

  public boolean elevatorAtAlgaeReefL3(boolean yellow) {
    if (!yellow) {
      return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorReef3Height
          - ElevatorConstants.kElevatorTolerance))
          && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorReef3Height
              + ElevatorConstants.kElevatorTolerance)));
    } else {
      return ((getCurrentMotorPosition() >= (ElevatorConstants.kElevatorReef3FlipHeight
          - ElevatorConstants.kElevatorTolerance))
          && (getCurrentMotorPosition() <= (ElevatorConstants.kElevatorReef3FlipHeight
              + ElevatorConstants.kElevatorTolerance)));
    }
  }

  public double minConflictHeight(Rotation2d target_angle) {
    var deg = Math.abs(target_angle.getDegrees());
    var minHeight = -7 * Math.pow(deg, 0.7 / 2) + 50;
    if (minHeight < ElevatorConstants.kElevatorSafeHeightMin) {
      minHeight = ElevatorConstants.kElevatorSafeHeightMin;
    } else if (minHeight > ElevatorConstants.kElevatorSafeHeightMax) {
      minHeight = ElevatorConstants.kElevatorSafeHeightMax;
    }
    return minHeight;
  }

  public boolean targetInConflictZone(double target_position, Rotation2d target_angle) {
    return target_position < minConflictHeight(target_angle);
  }

  public boolean inDangerZone() {
    return targetInDangerZone(getCurrentMotorPosition());
  }

  public boolean atTarget(double tolerance_rotations) {
    return Math.abs(getCurrentMotorPosition() - motionMagicPositionControl.Position) < tolerance_rotations;
  }

  public void periodic() {
    // display encoder readings on dashboard
    if (TuningModeConstants.kElevatorTuning) {
      SmartDashboard.putNumber("current Elevator Position", getCurrentMotorPosition());
    }
    SmartDashboard.putBoolean("top LS hit", topLimitSwitch.get());
    SmartDashboard.putBoolean("bottom LS hit", bottomLimitSwitch.get());
    SmartDashboard.putBoolean("zero LS hit", zeroLimitSwitch.get());

    if (TUNING_MODE) {
    }

    // the following code is for the drive speed multiplier based on the elevator
    // height
    double elevatorHeight = getCurrentMotorPosition();
    if (elevatorHeight < ElevatorConstants.kElevatorSafeTravelHeight) {

      // DriveState.getInstance().setElevatorMultiplier(1);
      DriveState.elevatorMultiplier = 1;
    } else {
      double multiplier = ((ElevatorConstants.kElevatorMinTravelHeight - elevatorHeight)
          / (ElevatorConstants.kElevatorMinTravelHeight
              - ElevatorConstants.kElevatorSafeTravelHeight));

      multiplier = Math.min(1, multiplier);
      multiplier = Math.max(0, multiplier);

      // driveState.setElevatorMultiplier(multiplier);
      DriveState.elevatorMultiplier = multiplier;
    }

    if (TuningModeConstants.kElevatorTuning) {
      SmartDashboard.putNumber("Elevator Multiplier", DriveState.elevatorMultiplier);
    }
  }

  // turn off elevator (stops motor all together)
  public void stop() {
    m_elevatorMotor.set(0);
  }

  public Command stopCommand() {
    return new RunCommand(() -> {
      m_elevatorMotor.set(0);
    }, this);
  }

  // open loop move elevator up & down but don't allow movement if the limit
  // switch is being hit
  public void moveDown() {

    // We are going down but bottom limit is not tripped so go at commanded speed
    m_elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  public Command moveDownCommand() {
    return new RunCommand(() -> {
      m_elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
    }, this);
  }

  public void zeroMoveUp() {

    // Go slower when we are zeroing
    m_elevatorMotor.set(.15);
  }

  public void moveUp() {

    m_elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  public void holdHeight() {
    // double targetPosition = getCurrentHeight();
    double targetPosition = getCurrentMotorPosition();
    setPos(targetPosition);
  }

}