// ==========================================================
//  Arm.java
//  Hippo Arm Project — CANcoder-Based Closed-Loop Control
// ----------------------------------------------------------
//  Uses the CANcoder on the arm pivot shaft as feedback.
//  The TalonFX drives the Kraken X60 until the CANcoder
//  reports the desired arm angle (in degrees).
//  Includes legacy method names for older commands.
// ==========================================================

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    // ----------------------------------------------------------
    // Hardware
    // ----------------------------------------------------------
    private final TalonFX motor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(Constants.Arm.ARM_ENCODER_ID);

    // Control requests
    private final DutyCycleOut manualDuty = new DutyCycleOut(0.0);
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.0);

    // ----------------------------------------------------------
    // Constructor
    // ----------------------------------------------------------
    public Arm() {

        // 1. Factory reset the motor
        motor.getConfigurator().apply(new TalonFXConfiguration());

        // 2. Build ONE unified configuration block
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // -----------------------------
        // Feedback (CANcoder)
        // -----------------------------
        cfg.Feedback.FeedbackRemoteSensorID = Constants.Arm.ARM_ENCODER_ID;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // -----------------------------
        // PID slot 0
        // -----------------------------
        cfg.Slot0.kP = Constants.Arm.ARM_kP;
        cfg.Slot0.kI = Constants.Arm.ARM_kI;
        cfg.Slot0.kD = Constants.Arm.ARM_kD;

        // -----------------------------
        // Neutral mode
        // -----------------------------
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // -----------------------------
        // Inversion (Phoenix 6 method)
        // -----------------------------
        cfg.MotorOutput.Inverted = Constants.Arm.ARM_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // -----------------------------
        // Software motion limits
        // -----------------------------
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        double minRot = 0.0; // 0° = horizontal
        double maxRot = 180.0 / 360.0; // 180° = half rotation

        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRot;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRot;

        // -----------------------------
        // Apply all configs at once
        // -----------------------------
        StatusCode sc = motor.getConfigurator().apply(cfg);
        if (!sc.isOK()) {
            System.out.println("[Arm] TalonFX config failed: " + sc.toString());
        } else {
            System.out.println("[Arm] Firmware soft limits enabled (0°–180°).");
        }

        System.out.println("[Arm] Initialized using CANcoder feedback.");
    }

    // ----------------------------------------------------------
    // Manual open-loop control (joystick testing)
    // ----------------------------------------------------------
    public void setPercent(double pct) {

        // Deadband
        if (Math.abs(pct) < Constants.Arm.JOYSTICK_DEADBAND) {
            pct = 0.0;
        }

        // Clamp to ±1 and scale max output
        pct = Math.max(-1.0, Math.min(1.0, pct)) * Constants.Arm.ARM_MAX_PERCENT_OUTPUT;

        manualDuty.Output = pct;
        motor.setControl(manualDuty);
    }

    // ----------------------------------------------------------
    // Closed-loop position control
    // ----------------------------------------------------------
    /** Move the arm until the CANcoder measures the requested angle. */
    public void moveToAngle(double targetDegrees) {

        // Clamp to software limits
        double MIN_DEG = 0.0;
        double MAX_DEG = 180.0;
        targetDegrees = Math.max(MIN_DEG, Math.min(MAX_DEG, targetDegrees));

        // Convert degrees → rotations
        double targetRotations = targetDegrees / 360.0;

        positionRequest.Position = targetRotations;
        motor.setControl(positionRequest);

        SmartDashboard.putNumber("Arm Target (rot)", targetRotations);
        SmartDashboard.putNumber("Arm Target (deg clamped)", targetDegrees);
    }

    // ----------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------
    /** Return the current arm angle in degrees. */
    public double getArmAngleDegrees() {
        double rotations = cancoder.getAbsolutePosition().getValueAsDouble();
        return rotations * 360.0;
    }

    /** Stop the motor immediately. */
    public void stop() {
        motor.setControl(new DutyCycleOut(0.0));
    }

    // ----------------------------------------------------------
    // Backward-compatibility aliases for older commands
    // ----------------------------------------------------------
    public void setTargetDegrees(double degrees) {
        moveToAngle(degrees);
    }

    public double getArmAngle() {
        return getArmAngleDegrees();
    }

    // ----------------------------------------------------------
    // Telemetry
    // ----------------------------------------------------------
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle (deg)", getArmAngleDegrees());
        SmartDashboard.putNumber("Arm CANcoder Rot", cancoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Motor Output %", motor.getDutyCycle().getValueAsDouble());

        boolean atLimit = getArmAngleDegrees() <= 0.5 || getArmAngleDegrees() >= 179.5;
        SmartDashboard.putBoolean("Arm At Limit", atLimit);
    }
}
