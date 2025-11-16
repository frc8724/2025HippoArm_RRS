package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * VisionAlign
 * -----------
 * Rotates the robot to face an AprilTag using PhotonVision yaw angle.
 *
 * • Translation is field-centric (so holding alignment does not drift)
 * • Rotation is robot-centric (so turning is intuitive + stable)
 * • Uses a simple proportional controller (P-only loop)
 * • Includes deadband, max rotational speed, and optional strafe assist
 *
 * This is the "real" post-debug version to replace VisionAlign_Debug.
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    /**
     * The swerve request we will send each cycle.
     * Phoenix 6 swerve requests are immutable, so we reassign this
     * each time we change velocity or rotation.
     */
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // ----------------------------------------------------------
    // TUNING CONSTANTS — SAFE STARTING VALUES
    // ----------------------------------------------------------

    /** Rotational P gain. Larger = faster turning. */
    private static final double kRotP = 0.02;

    /** Maximum allowed rotation speed (rad/sec). */
    private static final double kMaxRot = 1.5;

    /** Ignore yaw error within this many degrees (reduces jitter). */
    private static final double kYawDeadband = 1.5;

    /**
     * Optional sideways assist.
     * If you want the robot to slide left/right to center on the tag,
     * set this to around 0.15–0.25.
     *
     * For now we leave it off.
     */
    private static final double kStrafeAssist = 0.0;

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Required so this command interrupts TeleopDrive
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized");
    }

    @Override
    public void execute() {

        // ------------------------------------------------------
        // Read vision data from VisionSubsystem
        // ------------------------------------------------------
        boolean hasTarget = vision.hasTarget();
        double yaw = vision.getTargetYaw().getDegrees();
        // If desired later, switch to getSmoothedTargetYaw()

        System.out.println("[VisionAlign] hasTarget=" + hasTarget + " yaw=" + yaw);

        // ------------------------------------------------------
        // If no target, stop movement and return early
        // ------------------------------------------------------
        if (!hasTarget) {
            request = request
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
            drivetrain.setControl(request);
            return;
        }

        // ------------------------------------------------------
        // Rotation Control — P-only loop
        // ------------------------------------------------------
        // Convention:
        // +Yaw means the tag is LEFT → robot must rotate CCW (positive rate)
        double rotCmd = kRotP * yaw;

        // Deadband: avoid twitching near zero
        if (Math.abs(yaw) < kYawDeadband) {
            rotCmd = 0;
        }

        // Clamp rotational rate to safe values (rad/sec)
        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));

        // ------------------------------------------------------
        // Optional strafe assist (centering left/right)
        // ------------------------------------------------------
        double strafeCmd = kStrafeAssist * Math.signum(yaw);

        // ------------------------------------------------------
        // Build the swerve request for this cycle
        // ------------------------------------------------------
        request = request
                .withVelocityX(0) // no forward/back movement
                .withVelocityY(strafeCmd) // optional lateral assist
                .withRotationalRate(rotCmd);

        // ------------------------------------------------------
        // Send control to drivetrain
        // ------------------------------------------------------
        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign] End");

        // Hard stop to prevent drift when releasing button
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}
