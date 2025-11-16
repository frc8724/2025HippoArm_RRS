package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * VisionAlign (Step C)
 * --------------------
 * This version performs full AprilTag alignment by:
 *
 * 1) ROTATING to face the tag (using yaw error)
 * 2) STRAFING left/right to center the tag in the camera (using lateral offset)
 *
 * Notes:
 * - Rotation uses a proportional controller based on tag yaw.
 * - Strafe uses tag translation (meters-left/right from the camera).
 * - Translation data comes from PhotonVision: target.getBestCameraToTarget()
 * - Field-centric mode keeps sideways motion consistent with the field.
 * - Robot-centric rotation keeps turning intuitive and stable.
 *
 * This is the “true auto-center” behavior most teams want for reef scoring.
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    /**
     * The swerve request we build each cycle.
     * Phoenix 6 swerve requests are IMMUTABLE.
     * That means every call to .withVelocityX/Y/Rotation returns a NEW object.
     * So we reassign this request every loop.
     */
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // ----------------------------------------------------------
    // ROTATION TUNING CONSTANTS
    // ----------------------------------------------------------

    /** Rotational proportional gain. Higher = faster turning. */
    private static final double kRotP = 0.02;

    /** Maximum rotational speed in rad/sec (1.5 = safe starting point). */
    private static final double kMaxRot = 1.5;

    /** Ignore yaw error smaller than this many degrees to avoid jitter. */
    private static final double kYawDeadband = 1.5;

    // ----------------------------------------------------------
    // STRAFE TUNING CONSTANTS
    // ----------------------------------------------------------

    /**
     * Strafe proportional gain.
     * Units: (m/s output) = kStrafeP × (meters of lateral error)
     * Start around 2.0 for most FRC drivetrains.
     */
    private static final double kStrafeP = 2.0;

    /** Ignore strafe error smaller than this (meters). 0.02 m ≈ 2 cm. */
    private static final double kStrafeDeadband = 0.02;

    /** Maximum allowable strafe speed (m/s). Prevents hard lateral jumps. */
    private static final double kMaxStrafe = 0.75;

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        /**
         * This line is CRITICAL.
         *
         * Without addRequirements(drivetrain):
         * - TeleopDrive continues running
         * - This command NEVER controls the wheels
         * - Robot appears unresponsive during alignment
         *
         * With addRequirements:
         * - Holding the align button INTERRUPTS TeleopDrive
         * - Our alignment logic takes full control
         */
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized");
    }

    @Override
    public void execute() {

        // ------------------------------------------------------
        // 1) Read vision data from the VisionSubsystem
        // ------------------------------------------------------

        boolean hasTarget = vision.hasTarget(); // Does PhotonVision see a tag?
        double yaw = vision.getTargetYaw().getDegrees(); // Angle left/right
        double lat = vision.getTargetLateralOffset(); // Tag sideways offset in METERS

        System.out.println("[VisionAlign] hasTarget=" + hasTarget +
                " yaw(deg)=" + yaw +
                " lateral(m)=" + lat);

        // ------------------------------------------------------
        // 2) If the tag is lost, STOP (no blind drifting!)
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
        // 3) ROTATION CONTROL (turn to face the tag)
        // ------------------------------------------------------

        /**
         * PhotonVision yaw:
         * +YAW = tag is LEFT of robot → rotate CCW (positive rotCmd)
         * -YAW = tag is RIGHT → rotate CW (negative rotCmd)
         *
         * rotCmd = P × error
         */
        double rotCmd = kRotP * yaw;

        // Deadband prevents jitter when nearly centered
        if (Math.abs(yaw) < kYawDeadband)
            rotCmd = 0;

        // Clamp rotation for safety + stability
        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));

        // ------------------------------------------------------
        // 4) STRAFE CONTROL (center horizontally on the tag)
        // ------------------------------------------------------

        /**
         * PhotonVision camera-to-target transform:
         *
         * transform.getY() = sideways distance in METERS
         * +Y = tag is LEFT
         * -Y = tag is RIGHT
         *
         * Perfect for lateral centering.
         */
        double strafeCmd = kStrafeP * lat;

        // Deadband so robot doesn't micro-wiggle when perfectly aligned
        if (Math.abs(lat) < kStrafeDeadband)
            strafeCmd = 0;

        // Clamp final strafe command
        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));

        // ------------------------------------------------------
        // 5) BUILD THE FINAL SWERVE REQUEST
        // ------------------------------------------------------

        /**
         * Why field-centric?
         * - Sideways motion remains sideways relative to the field.
         * - Robot rotation does NOT change translation direction.
         *
         * Why robot-centric rotation?
         * - Rotational rate is always about the robot’s own Z axis.
         */
        request = request
                .withVelocityX(0) // no forward/back movement yet
                .withVelocityY(strafeCmd) // STRAFE left/right to center
                .withRotationalRate(rotCmd); // ROTATE to face the tag

        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign] End");

        // Hard stop when the user releases the alignment button
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}
