package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TuningModeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.MotorDefaultsConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_deployMotor = new TalonFX(Constants.CanIdConstants.kIntakeDeployCanId, "rio");
    private StatusSignal<Angle> deployPosition;
    // private StatusSignal<AngularVelocity> pivotArmVelocity;
    TalonFXConfiguration config = new TalonFXConfiguration();

    private final SparkMax m_outerRoller = new SparkMax(CanIdConstants.kIntakeOuterRollerCanId,
            MotorDefaultsConstants.NeoMotorType);

    private final SparkMax m_innerRoller = new SparkMax(CanIdConstants.kIntakeInnerRollerCanId,
            MotorDefaultsConstants.NeoMotorType);

    // The gyro sensor
    // private final Pigeon2 m_pivotGyro = new CANcoder()

    private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

    public IntakeSubsystem() {

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.kIntakeDeployP;
        slot0Configs.kI = IntakeConstants.kIntakeDeployI;
        slot0Configs.kD = IntakeConstants.kIntakeDeployD;
        slot0Configs.kV = IntakeConstants.kIntakeDeployV;
        slot0Configs.kA = IntakeConstants.kIntakeDeployA;
        slot0Configs.kS = IntakeConstants.kIntakeDeployS;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotArmConstants.kPivotArmMMVelo;
        config.MotionMagic.MotionMagicAcceleration = Constants.PivotArmConstants.kPivotArmMMAcc;
        config.MotionMagic.MotionMagicJerk = Constants.PivotArmConstants.kPivotArmMMJerk;

        deployPosition = m_deployMotor.getPosition();

        m_deployMotor.getConfigurator().apply(config);
        m_deployMotor.getConfigurator().apply(slot0Configs);

    }

    public Rotation2d getDeployRotation() {
        deployPosition.refresh();
        return Rotation2d.fromDegrees(deployPosition.getValueAsDouble());
    }

    public void setDeployRotation(Rotation2d rotation) {
        motionMagicPositionControl.Position = rotation.getRotations();
        m_deployMotor.setControl(motionMagicPositionControl);
    }

    public boolean isAtIntakeAngle(Rotation2d tolerance) {
        return Math.abs(getDeployRotation().getRotations()
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

    public void enableRollers() {
        this.setInnerRollerSpeed(IntakeConstants.kIntakeInnerSpeed);
        this.setOuterRollerSpeed(IntakeConstants.kIntakeSpeed);
    }

    public void disableRollers() {
        this.setInnerRollerSpeed(0);
        this.setOuterRollerSpeed(0);
    }

    public void ejectRollers() {
        // this.setInnerRollerSpeed(-IntakeConstants.kIntakeInnerSpeed);
        this.setOuterRollerSpeed(-IntakeConstants.kIntakeSpeed);
    }

    public void periodic() {
        if (TuningModeConstants.kPivotTuning) {
            // Get motor readings
            // SmartDashboard.putNumber("currentPosition", getCurrentPosition());
            SmartDashboard.putNumber("currentIntakeAngle", getDeployRotation().getDegrees());
            // SmartDashboard.putNumber("current Gyro Roll", getPivotArmAngle());
        }
    }

}
