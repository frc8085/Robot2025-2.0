package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX m_pivotMotor = new TalonFX(PivotArmConstants.kPivotArmCanId, "rio");
    private final CANcoder m_pivotEncoder = new CANcoder(PivotArmConstants.kPivotArmCancoderCanId, "rio");

    private StatusSignal<Angle> pivotArmPosition;
    private StatusSignal<AngularVelocity> pivotArmVelocity;

    private StatusSignal<Angle> pivotEncoderPosition;

    CANcoderConfiguration m_pivotEncoderConfig = new CANcoderConfiguration();
    TalonFXConfiguration m_pivotMotorConfig = new TalonFXConfiguration();

    // The gyro sensor
    private final Pigeon2 m_pivotGyro = new Pigeon2(PivotArmConstants.kPivotGyroCanId);

    private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

    private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);

    public PivotSubsystem() {

        /* Configure CANcoder to zero the magnet appropriately */
        m_pivotEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        m_pivotEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_pivotEncoderConfig.MagnetSensor.MagnetOffset = PivotArmConstants.kPivotCancoderOffset;
        m_pivotEncoder.getConfigurator().apply(m_pivotEncoderConfig);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = PivotArmConstants.kPivotArmP;
        slot0Configs.kI = PivotArmConstants.kPivotArmI;
        slot0Configs.kD = PivotArmConstants.kPivotArmD;
        slot0Configs.kV = PivotArmConstants.kPivotArmV;
        slot0Configs.kA = PivotArmConstants.kPivotArmA;
        // slot0Configs.kS = PivotArmConstants.kPivotArmS;

        // new stuff for Synced Cancoder
        m_pivotMotorConfig.Feedback.FeedbackRemoteSensorID = m_pivotEncoder.getDeviceID();
        m_pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        m_pivotMotorConfig.Feedback.SensorToMechanismRatio = 3.0;
        m_pivotMotorConfig.Feedback.RotorToSensorRatio = 9.0;

        m_pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //

        m_pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = PivotArmConstants.kPivotArmMMVelo;
        m_pivotMotorConfig.MotionMagic.MotionMagicAcceleration = PivotArmConstants.kPivotArmMMAcc;
        m_pivotMotorConfig.MotionMagic.MotionMagicJerk = PivotArmConstants.kPivotArmMMJerk;

        pivotArmPosition = m_pivotMotor.getPosition();
        pivotArmVelocity = m_pivotMotor.getVelocity();
        pivotEncoderPosition = m_pivotEncoder.getPosition();

        m_pivotMotor.getConfigurator().apply(m_pivotMotorConfig);
        m_pivotMotor.getConfigurator().apply(slot0Configs);

    }

    public double getPivotArmAngle() {
        return Rotation2d.fromDegrees(m_pivotGyro.getPitch().getValueAsDouble()).getDegrees();
    }

    public boolean atHomeAngle() {
        if ((getPivotArmAngle()) == -25) {
            return true;
        } else {
            return false;
        }
    }

    public boolean lessThanHomeAngle() {
        if ((getPivotArmAngle()) < -25) {
            return true;

        } else {
            return false;
        }
    }

    public boolean moreThanHomeAngle() {
        if ((getPivotArmAngle()) >= -25) {
            return true;
        } else {
            return false;
        }
    }

    // Pivot Angle Checks
    public boolean pivotAtAlgaeYellowScorePosition() {
        return getCurrentRotation().getDegrees() <= PivotArmConstants.kPivotAlgaeNetYellow
                + PivotArmConstants.kPivotTolerance.getDegrees();
    }

    public boolean pivotAtAlgaeBlueScorePosition() {
        return (getCurrentRotation().getDegrees() <= PivotArmConstants.kPivotAlgaeNetBlue
                + PivotArmConstants.kPivotTolerance.getDegrees())
                && (getCurrentRotation().getDegrees() >= PivotArmConstants.kPivotAlgaeNetBlue
                        - PivotArmConstants.kPivotTolerance.getDegrees());
    }

    public boolean pivotAtHomeAngle() {
        return ((getCurrentRotation().getDegrees() <= (-PivotArmConstants.kPivotHome
                + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                (getCurrentRotation().getDegrees() >= (-PivotArmConstants.kPivotHome
                        - PivotArmConstants.kPivotTolerance.getDegrees())));
    }

    public boolean pivotAtCoralDropOffAngle(boolean yellow) {
        if (!yellow) {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotCoralDropOff
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotCoralDropOff
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        } else {
            return ((getCurrentRotation().getDegrees() <= (-PivotArmConstants.kPivotCoralDropOff
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (-PivotArmConstants.kPivotCoralDropOff
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        }
    }

    public boolean pivotAtCoral1DropOffAngle(boolean yellow) {
        if (!yellow) {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotCoralDropOff1
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotCoralDropOff1
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        } else {
            return ((getCurrentRotation().getDegrees() <= (-PivotArmConstants.kPivotCoralDropOff1
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (-PivotArmConstants.kPivotCoralDropOff1
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        }
    }

    public boolean pivotAtAlgaeReef2DropOffAngle(boolean yellow) {
        if (!yellow) {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotReef
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotReef
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        } else {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotReef2Flip
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotReef2Flip
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        }
    }

    public boolean pivotAtAlgaeReef3DropOffAngle(boolean yellow) {
        if (!yellow) {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotReef
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotReef
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        } else {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotReef3Flip
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotReef3Flip
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        }
    }

    public boolean pivotAtCoral4DropOffAngle(boolean yellow) {
        if (!yellow) {
            return ((getCurrentRotation().getDegrees() <= (PivotArmConstants.kPivotCoralDropOff4
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (PivotArmConstants.kPivotCoralDropOff4
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        } else {
            return ((getCurrentRotation().getDegrees() <= (-PivotArmConstants.kPivotCoralDropOff4
                    + PivotArmConstants.kPivotTolerance.getDegrees())) &&
                    (getCurrentRotation().getDegrees() >= (-PivotArmConstants.kPivotCoralDropOff4
                            - PivotArmConstants.kPivotTolerance.getDegrees())));
        }
    }

    private Rotation2d motorPosToAngle(double pos) {
        return Rotation2d.fromRotations(pos / PivotArmConstants.kPivotMotorGearRatio);
    }

    private double angleToMotorPos(Rotation2d angle) {
        return angle.getRotations() * PivotArmConstants.kPivotMotorGearRatio;
    }

    public void setPos(Rotation2d angle) {

        if (angle.getRotations() < PivotArmConstants.kPivotArmMin.getRotations()) {
            angle = PivotArmConstants.kPivotArmMin;
        } else if (angle.getRotations() > PivotArmConstants.kPivotArmMax.getRotations()) {
            angle = PivotArmConstants.kPivotArmMax;
        }
        this.setPosManual(angle);
    }

    // WARNING: NO BOUNDS ON THIS FUNCTION, ONLY MANUAL USE ONLY, NOT FOR COMMANDS
    public void setPosManual(Rotation2d angle) {

        if (angle.getRotations() < PivotArmConstants.kPivotArmMinManual.getRotations()) {
            angle = PivotArmConstants.kPivotArmMinManual;
        } else if (angle.getRotations() > PivotArmConstants.kPivotArmMaxManual.getRotations()) {
            angle = PivotArmConstants.kPivotArmMaxManual;
        }

        motionMagicPositionControl.Position = angleToMotorPos(angle);
        motionMagicPositionControl.FeedForward = Math.sin(angle.getRadians())
                * PivotArmConstants.kPivotArmFF;
        m_pivotMotor.setControl(motionMagicPositionControl);
        SmartDashboard.putNumber("rotation2d value", angle.getRotations());

    }

    public void setRotorPos(Rotation2d angle) {
        m_pivotMotor.setPosition(angle.getRotations());
    }

    public void setAnglePos(Rotation2d angle) {
        m_pivotMotor.setPosition(angle.getRotations() * PivotArmConstants.kPivotMotorGearRatio);
    }

    public double getCurrentPosition() {
        pivotArmPosition.refresh();
        return pivotArmPosition.getValueAsDouble();
    }

    public double getCurrentEncoderPosition() {
        pivotEncoderPosition.refresh();
        return pivotEncoderPosition.getValueAsDouble();
    }

    public double getCurrentVelocity() {
        pivotArmVelocity.refresh();
        return pivotArmVelocity.getValueAsDouble();
    }

    public boolean isAtTarget(Rotation2d tolerance) {
        return Math.abs(getCurrentRotation().getRotations()
                - motorPosToAngle(motionMagicPositionControl.Position).getRotations()) < tolerance.getRotations();

    }

    public Rotation2d getCurrentRotation() {
        return motorPosToAngle(getCurrentPosition());
    }

    public void periodic() {
        if (TuningModeConstants.kPivotTuning) {
            // Get motor readings
            // SmartDashboard.putNumber("currentPosition", getCurrentPosition());
            SmartDashboard.putNumber("currentAngle", getCurrentRotation().getDegrees());
            // SmartDashboard.putNumber("current Gyro Roll", getPivotArmAngle());
            SmartDashboard.putNumber("CancoderReading", getCurrentEncoderPosition());
        }

    }

    public void zeroStart() {
        m_pivotMotor.set(0.04);
    }

    public void zeroReverse() {
        m_pivotMotor.set(-0.04);
    }

    public void start() {
        m_pivotMotor.set(PivotArmConstants.kPivotArmSpeed);
    }

    public void reverse() {
        m_pivotMotor.set(-PivotArmConstants.kPivotArmSpeed);
    }

    public void stop() {
        m_pivotMotor.set(0);
    }

    // checks whether the pivot arm is in the danger zone for the elevator at target
    // angle GOOD
    public boolean targetInDangerZone(Rotation2d targetAngle) {
        return targetAngle.getDegrees() > PivotArmConstants.kPivotArmSwingThroughMin.getDegrees()
                + PivotArmConstants.kPivotTolerance.getDegrees()
                && targetAngle.getDegrees() < PivotArmConstants.kPivotArmSwingThroughMax.getDegrees()
                        - PivotArmConstants.kPivotTolerance.getDegrees();
    }

    public boolean inDangerZone() {
        return targetInDangerZone(getCurrentRotation());
    }

    public boolean willPivotThroughDangerZone(Rotation2d targetAngle) {
        var result = false;

        // check if the start and/or end position is in the danger zone
        if (this.targetInDangerZone(targetAngle) || this.inDangerZone()) {
            result = true;
        }

        // check if the pivot will pass through the danger zone
        // AKA: if the current position is between the target and the danger zone

        // check if the current position is on one side of the danger zone, and the
        // target is on the other
        if (Math.signum(getCurrentRotation().getDegrees()) != Math.signum(targetAngle.getDegrees())) {
            result = true;
        }

        return result;
    }

    public void holdPivotArm() {
        Rotation2d targetAngle = getCurrentRotation();
        setPos(targetAngle);
    }

    public void holdPivotArmManual() {
        Rotation2d targetAngle = getCurrentRotation();
        setPosManual(targetAngle);
    }

}
