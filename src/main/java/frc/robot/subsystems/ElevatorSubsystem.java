package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.States.DriveState;

public class ElevatorSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = TuningModeConstants.kElevatorTuning;

  // CAN ID and CANbus
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIdConstants.kElevatorCanId, "rio");
  TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoder m_elevatorEncoder = new CANcoder(Constants.CanIdConstants.kElevatorCancoderCanID);

  // get encoder data
  private StatusSignal<Angle> elevatorPosition;
  private StatusSignal<AngularVelocity> elevatorVelocity;

  // private driveState = new DriveState.getInstance();

  private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

  private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);

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

    // Setting Motor Configs
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Setting Motion Magic Configs
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorMMVelo;
    config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kElevatorMMAcc;
    config.MotionMagic.MotionMagicJerk = Constants.ElevatorConstants.kElevatorMMJerk;

    // get sensor readings
    elevatorPosition = m_elevatorMotor.getPosition();
    elevatorVelocity = m_elevatorEncoder.getVelocity();

    // config.HardwareLimitSwitch.ForwardLimitSource =
    // ForwardLimitSourceValue.LimitSwitchPin;
    // config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 0;
    // config.HardwareLimitSwitch.ForwardLimitEnable = true;
    // config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;

    // Apply Configs
    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  // private double rotationsToMotorPosition(double rotations) {
  // return rotations * Constants.ElevatorConstants.kElevatorMotorGearRatio;
  // }

  // private double motorPositionToInches(double motorPosition) {
  // return motorPosition / Constants.ElevatorConstants.kElevatorMotorGearRatio;
  // }

  // Setting the height of the elevator
  // public void setPos(double rotations) {
  public void setPos(double rotations) {

    // Checks to see if the elevator height is below the minimum, and if it is, set
    // it to the minimum and
    // check to see if the elevator height is above the maximum, and if it is, set
    // it to the maximum

    if ((rotations < Constants.ElevatorConstants.kElevatorMin)) {
      rotations = Constants.ElevatorConstants.kElevatorMin;
    } else if ((rotations > Constants.ElevatorConstants.kElevatorMax)) {
      rotations = Constants.ElevatorConstants.kElevatorMax;
    }

    // set the feedforward value based on the elevator height
    double ff = 0;
    if (rotations > ElevatorConstants.kElevatorStage2Height) {
      ff = ElevatorConstants.kElevatorStage3FF;
    } else if (rotations > Constants.ElevatorConstants.kElevatorStage1Height) {
      ff = ElevatorConstants.kElevatorStage2FF;
    }

    // motionMagicControl.Position = rotationsToMotorPosition(rotations);
    motionMagicPositionControl.Position = rotations;
    motionMagicVelocityControl.FeedForward = ff;
    m_elevatorMotor.setControl(motionMagicPositionControl);
    // SmartDashboard.putNumber("height value", rotations);
    // SmartDashboard.putNumber("motor value", rotationsToMotorPosition(rotations));
  }

  // set the zero value of the motor encoder
  // public void zero(double rotations) {
  // m_elevatorMotor.setPosition(
  // rotations * Constants.ElevatorConstants.kElevatorMotorGearRatio);
  // }

  public void zero() {
    m_elevatorMotor.setPosition(
        Constants.ElevatorConstants.kElevatorStage1Height);
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

  // // translate the current elevator encoder position into elevator height
  // public double getCurrentHeight() {
  // // return (motorPositionToInches(getCurrentMotorPosition()));
  // return getCurrentMotorPosition();
  // }

  public boolean targetInDangerZone(double target_position) {
    return target_position < (Constants.ElevatorConstants.kElevatorSafeHeightMax);
  }

  public boolean elevatorAtAlgaeScoreHeight() {
    return getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorNetHeight
        - Constants.ElevatorConstants.kElevatorTolerance);
  }

  public boolean elevatorAtCoralDropOff1Height() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorCoralDropOff1Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorCoralDropOff1Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff2Height() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorCoralDropOff2Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorCoralDropOff2Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff3Height() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorCoralDropOff3Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorCoralDropOff3Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtCoralDropOff4Height() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorCoralDropOff4Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorCoralDropOff4Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtAlgaeReefL2() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorReef2Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorReef2Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public boolean elevatorAtAlgaeReefL3() {
    return ((getCurrentMotorPosition() >= (Constants.ElevatorConstants.kElevatorReef3Height
        - Constants.ElevatorConstants.kElevatorTolerance))
        && (getCurrentMotorPosition() <= (Constants.ElevatorConstants.kElevatorReef3Height
            + Constants.ElevatorConstants.kElevatorTolerance)));
  }

  public double minConflictHeight(Rotation2d target_angle) {
    var deg = Math.abs(target_angle.getDegrees());
    var minHeight = -7 * Math.pow(deg, 0.7 / 2) + 50;
    if (minHeight < Constants.ElevatorConstants.kElevatorSafeHeightMin) {
      minHeight = Constants.ElevatorConstants.kElevatorSafeHeightMin;
    } else if (minHeight > Constants.ElevatorConstants.kElevatorSafeHeightMax) {
      minHeight = Constants.ElevatorConstants.kElevatorSafeHeightMax;
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