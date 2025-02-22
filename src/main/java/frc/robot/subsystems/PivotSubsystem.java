package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotArmConstants;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX m_pivotMotor = new TalonFX(Constants.CanIdConstants.kPivotArmCanId, "rio");
    private StatusSignal<Angle> pivotArmPosition;
    private StatusSignal<AngularVelocity> pivotArmVelocity;
    TalonFXConfiguration config = new TalonFXConfiguration();
    // private final CANcoder m_pivotEncoder = new
    // CANcoder(Constants.CanIdConstants.kPivotArmCancoderCanID);

    private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

    private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);

    public PivotSubsystem() {

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = PivotArmConstants.kPivotArmP;
        slot0Configs.kI = PivotArmConstants.kPivotArmI;
        slot0Configs.kD = PivotArmConstants.kPivotArmD;
        slot0Configs.kV = PivotArmConstants.kPivotArmV;
        slot0Configs.kA = PivotArmConstants.kPivotArmA;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotArmConstants.kPivotArmMMVelo;
        config.MotionMagic.MotionMagicAcceleration = Constants.PivotArmConstants.kPivotArmMMAcc;
        config.MotionMagic.MotionMagicJerk = Constants.PivotArmConstants.kPivotArmMMJerk;

        // expirementing with encoder
        // config.Feedback.
        // config.Feedback.FeedbackRemoteSensorID =
        // Constants.CanIdConstants.kPivotArmCancoderCanID;
        // config.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RemoteCANcoder;
        // config.Feedback.RotorToSensorRatio = 9. / 1;
        // config.Feedback.SensorToMechanismRatio = 3. / 1;
        // config.Feedback.FeedbackRotorOffset = 0;

        // CANcoderConfiguration c = new CANcoderConfiguration();
        // c.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // m_pivotEncoder.getConfigurator().apply(c);

        pivotArmPosition = m_pivotMotor.getPosition();
        pivotArmVelocity = m_pivotMotor.getVelocity();

        m_pivotMotor.getConfigurator().apply(config);
        m_pivotMotor.getConfigurator().apply(slot0Configs);

    }

    private Rotation2d motorPosToAngle(double pos) {
        return Rotation2d.fromRotations(pos / Constants.PivotArmConstants.kPivotMotorGearRatio);
    }

    private double angleToMotorPos(Rotation2d angle) {
        return angle.getRotations() * Constants.PivotArmConstants.kPivotMotorGearRatio;
    }

    public void setPos(Rotation2d angle) {

        if (angle.getRotations() < Constants.PivotArmConstants.kPivotArmMin.getRotations()) {
            angle = Constants.PivotArmConstants.kPivotArmMin;
        } else if (angle.getRotations() > Constants.PivotArmConstants.kPivotArmMax.getRotations()) {
            angle = Constants.PivotArmConstants.kPivotArmMax;
        }

        // motionMagicControl.Position = angle.getRotations() *
        // Constants.PivotArmConstants.kPivotMotorGearRatio;
        motionMagicPositionControl.Position = angleToMotorPos(angle);
        m_pivotMotor.setControl(motionMagicPositionControl);
        SmartDashboard.putNumber("rotation2d value", angle.getRotations());
    }

    public void setRotorPos(Rotation2d angle) {
        m_pivotMotor.setPosition(angle.getRotations());
    }

    public double getCurrentPosition() {
        pivotArmPosition.refresh();
        return pivotArmPosition.getValueAsDouble();
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
        // Get motor readings
        SmartDashboard.putNumber("currentPosition", getCurrentPosition());
        SmartDashboard.putNumber("currentAngle", getCurrentRotation().getDegrees());

        // SmartDashboard.putNumber("Pivot Deg", getCurrentPosition() * 360); // the
        // getPosition function accounts for
        // // changes in configs (gear ratio)
        // // getRotorPosition just gets the motor
        // // value.
        // SmartDashboard.putNumber("Rotor Deg",
        // m_pivotMotor.getRotorPosition().getValueAsDouble() * 360);
        // SmartDashboard.putNumber("Encoder Deg",
        // m_pivotEncoder.getPosition().getValueAsDouble() * 360);

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

    public void keepPivot(Rotation2d positionPivot) {

        setPos(positionPivot);

    }

    // checks whether the pivot arm is in the danger zone for the elevator at target
    // angle
    public boolean targetInDangerZone(Rotation2d targetAngle) {
        return targetAngle.getDegrees() < Constants.PivotArmConstants.kPivotArmSwingThroughMin.getDegrees()
                && targetAngle.getDegrees() > Constants.PivotArmConstants.kPivotArmSwingThroughMax.getDegrees();
    }

    public boolean inDangerZone() {
        return targetInDangerZone(getCurrentRotation());
    }

    public boolean willPivotThroughDangerZone(Rotation2d targetAngle) {
        var result = false;

        // check if the start and/or end position is in the danger zone
        if (targetInDangerZone(targetAngle) || inDangerZone()) {
            result = true;
        }

        // check if the pivot will pass through the danger zone
        // AKA: if the current position is between the target and the danger zone

        // check if the current position is on one side of the danger zone, and the
        // target is on the other
        if ((getCurrentRotation().getDegrees() > Constants.PivotArmConstants.kPivotArmSwingThroughMin.getDegrees()
                && targetAngle.getDegrees() < Constants.PivotArmConstants.kPivotArmSwingThroughMax.getDegrees()) ||
                (getCurrentRotation().getDegrees() < Constants.PivotArmConstants.kPivotArmSwingThroughMax.getDegrees()
                        && targetAngle.getDegrees() > Constants.PivotArmConstants.kPivotArmSwingThroughMin
                                .getDegrees())) {
            result = true;
        }

        return result;
    }

    public void holdPivotArm() {
        Rotation2d targetAngle = getCurrentRotation();
        setPos(targetAngle);
    }

}
