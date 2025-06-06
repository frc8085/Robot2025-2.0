package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.subsystems.Pivot.PivotArmConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.MotorDefaultsConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_deployMotor = new TalonFX(IntakeConstants.kIntakeDeployCanId, "rio");
    private StatusSignal<Angle> deployPosition;
    // private StatusSignal<AngularVelocity> pivotArmVelocity;
    TalonFXConfiguration config = new TalonFXConfiguration();

    private final SparkMax m_outerRoller = new SparkMax(IntakeConstants.kIntakeOuterRollerCanId,
            MotorDefaultsConstants.NeoMotorType);

    private final SparkMax m_innerRoller = new SparkMax(IntakeConstants.kIntakeInnerRollerCanId,
            MotorDefaultsConstants.NeoMotorType);

    DigitalInput leftLightSensor = new DigitalInput(IntakeConstants.kLeftLightSensorDIO);

    DigitalInput rightLightSensor = new DigitalInput(IntakeConstants.kRightLightSensorDIO);// The gyro sensor
    // private final Pigeon2 m_pivotGyro = new CANcoder()

    private CANcoder m_intakeEncoder = new CANcoder(IntakeConstants.kIntakeDeployCanCoderCanId);

    CANcoderConfiguration m_intakeEncoderConfig = new CANcoderConfiguration();

    private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

    public IntakeSubsystem() {

        m_intakeEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        m_intakeEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_intakeEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.kCanCoderOffset;
        m_intakeEncoder.getConfigurator().apply(m_intakeEncoderConfig);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.kIntakeDeployP;
        slot0Configs.kI = IntakeConstants.kIntakeDeployI;
        slot0Configs.kD = IntakeConstants.kIntakeDeployD;
        slot0Configs.kV = IntakeConstants.kIntakeDeployV;
        slot0Configs.kA = IntakeConstants.kIntakeDeployA;
        slot0Configs.kS = IntakeConstants.kIntakeDeployS;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimit = 30;

        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kIntakeDeployMMVelo;
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kIntakeDeployMMAcc;
        config.MotionMagic.MotionMagicJerk = IntakeConstants.kIntakeDeployMMJerk;

        deployPosition = m_deployMotor.getPosition();

        m_deployMotor.getConfigurator().apply(config);
        m_deployMotor.getConfigurator().apply(slot0Configs);

    }

    public boolean getLeftLightSensor() {
        return leftLightSensor.get();
    }

    public boolean getRightLightSensor() {
        return rightLightSensor.get();
    }

    public boolean getAnyLightSensor() {
        return getLeftLightSensor() || getRightLightSensor();
    }

    public boolean hasCoralCentered() {
        return getLeftLightSensor() && getRightLightSensor();
    }

    public Rotation2d getDeployRotation() {
        deployPosition.refresh();
        return Rotation2d.fromRotations(deployPosition.getValueAsDouble());
    }

    public double getCanCoderMotorPosition() {
        double canPosition = this.m_intakeEncoder.getAbsolutePosition().getValueAsDouble();

        double offsetPosition = canPosition * IntakeConstants.kIntakeDeployGearRatio;

        // get the current motor position
        double motorPosition = IntakeConstants.kIntakeInAngle.getRotations();

        double currentMotorPosition = motorPosition + offsetPosition;

        return currentMotorPosition;
    }

    public void zeroIntake() {
        double currentMotorPosition = getCanCoderMotorPosition();

        // set the motor to the current motor position
        this.m_deployMotor.setPosition(currentMotorPosition);
    }

    public void setDeployRotation(Rotation2d rotation) {
        motionMagicPositionControl.Position = rotation.getRotations();
        m_deployMotor.setControl(motionMagicPositionControl);
    }

    public boolean isAtIntakeAngle(Rotation2d tolerance) {
        return Math.abs(this.getDeployRotation().getRotations()
                - motionMagicPositionControl.Position) < tolerance.getRotations();
    }

    public boolean isDeployed() {
        return getDeployRotation().getDegrees() > IntakeConstants.kIntakeOutAngle.getDegrees();
    }

    public void setOuterRollerSpeed(double speed) {
        m_outerRoller.set(speed);
    }

    public void setInnerRollerSpeed(double speed) {
        m_innerRoller.set(speed);
    }

    public void enableOuterRollers() {
        this.setOuterRollerSpeed(IntakeConstants.kIntakeSpeed);
    }

    public void idleOuterRollers() {
        this.setOuterRollerSpeed(0.1);
    }

    public void disableOuterRollers() {
        this.setOuterRollerSpeed(0);
    }

    public void enableInnerRollers() {
        this.setInnerRollerSpeed(-IntakeConstants.kIntakeInnerSpeed);
    }

    public void disableInnerRollers() {
        this.setInnerRollerSpeed(0);
    }

    public void enableRollers() {
        this.setInnerRollerSpeed(-IntakeConstants.kIntakeInnerSpeed);
        this.setOuterRollerSpeed(IntakeConstants.kIntakeSpeed);
    }

    public void disableRollers() {
        this.setInnerRollerSpeed(0);
        this.setOuterRollerSpeed(0);
    }

    public void idleRollers() {
        this.setInnerRollerSpeed(-IntakeConstants.kIntakeRestingSpeed);
    }

    public void ejectRollers() {
        // this.setInnerRollerSpeed(-IntakeConstants.kIntakeInnerSpeed);
        this.setOuterRollerSpeed(-IntakeConstants.kIntakeEjectSpeed);
    }

    public void ejectHandoffRollers() {
        // this.setInnerRollerSpeed(-IntakeConstants.kIntakeInnerSpeed);
        this.setOuterRollerSpeed(-IntakeConstants.kIntakeHandoffSpeed);
    }

    public void periodic() {
        if (TuningModeConstants.kPivotTuning) {
            // Get motor readings
            // SmartDashboard.putNumber("currentPosition", getCurrentPosition());
            SmartDashboard.putNumber("currentIntakeAngle", getDeployRotation().getRotations());
            SmartDashboard.putBoolean("hasCoral", this.getAnyLightSensor());
            SmartDashboard.putBoolean("leftCoral", this.getLeftLightSensor());
            SmartDashboard.putBoolean("rightCoral", this.getRightLightSensor());
            SmartDashboard.putNumber("Intake Cancoder Position",
                    m_intakeEncoder.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Intake Motor Position", this.getCanCoderMotorPosition());
            // SmartDashboard.putNumber("current Gyro Roll", getPivotArmAngle());
        }
    }

}
