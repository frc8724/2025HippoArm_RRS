package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

/**
 * VisionAlign — Side-Facing (Right-Side Scoring)
 * ----------------------------------------------
 * FULL COMMENTED VERSION FOR STUDENT UNDERSTANDING
 *
 * PURPOSE:
 *   Align the robot to an AprilTag on the reef using:
 *     1) Rotation (face the tag)
 *     2) Lateral alignment (strafe left/right)
 *     3) Forward movement (set scoring distance)
 *
 *   Designed for:
 *     - LL2+ camera (noisy, needs smoothing)
 *     - Side-mounted camera (offset from robot center)
 *     - Field-centric drivetrain (Phoenix 6)
 *
 * TIMELINE OF CONTROL:
 *   - Read raw photon data
 *   - Smooth it (low-pass filter)
 *   - Apply camera-offset compensation (so robot, not camera, squares)
 *   - Compute rotCmd, strafeCmd, driveCmd
 *   - Clamp values
 *   - Handle brief target loss
 *   - Apply swerve velocities
 *
 * ALL math is in robot-centric coordinates:
 *   +X = left/right strafe
 *   +Y = forward/back drive
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Phoenix 6 swerve request
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // ----------------------------------------------------------
    // CAMERA MOUNT OFFSETS
    // ----------------------------------------------------------
    // These convert camera-measured distances into robot-front distances.
    // YOU MUST update these if camera moves.

    /** Camera is 17.75 in behind front bumper = 0.45085 m */
    private static final double kCameraForwardOffsetMeters = 0.45085;

    /** Camera is 3.375 in LEFT of robot centerline = 0.08573 m */
    private static final double kCameraLateralOffsetMeters = 0.08573;

    // ----------------------------------------------------------
    // FILTERED VALUES (EMA smoothing)
    // ----------------------------------------------------------
    // LL2+ needs smoothing because of its sensor noise.

    private double filteredYaw = 0.0;   // degrees
    private double filteredLat = 0.0;   // meters
    private double filteredFwd = 0.0;   // meters

    // ----------------------------------------------------------
    // ROTATION CONTROL CONSTANTS
    // ----------------------------------------------------------

    private static final double kRotP = 0.040;     // P gain
    private static final double kMaxRot = 3.0;     // rad/s clamp
    private static final double kMinRot = 0.20;    // min speed to overcome friction
    private static final double kYawDeadband = 0.15; // degrees
    private static final double kRotSoftZoneDeg = 6.0; // degrees

    // ----------------------------------------------------------
    // STRAFE CONTROL CONSTANTS
    // ----------------------------------------------------------

    private static final double kStrafeP = 0.01;       // P gain
    private static final double kStrafeDeadband = 0.025;
    private static final double kMaxStrafe = 0.5;      // m/s cap

    // ----------------------------------------------------------
    // FORWARD DRIVE CONSTANTS
    // ----------------------------------------------------------

    /** Desired distance from front bumper to the reef. */
    private static final double kDesiredFrontDistanceMeters = 0.01;

    private static final double kDriveP = 0.95;
    private static final double kDriveDeadband = 0.03;
    private static final double kMaxDrive = 1.5;

    // ----------------------------------------------------------
    // LOST-TARGET FALLBACK MEMORY
    // ----------------------------------------------------------

    private double lastSeenTime = 0.0;
    private double lastDriveCmd = 0.0;
    private double lastStrafeCmd = 0.0;
    private double lastRotCmd = 0.0;

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    // ----------------------------------------------------------
    // initialize()
    // ----------------------------------------------------------

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] INIT");
        lastSeenTime = Timer.getFPGATimestamp();
    }

    // ----------------------------------------------------------
    // execute()
    // ----------------------------------------------------------

    @Override
    public void execute() {

        // ======================================================
        // 1) READ RAW PHOTONVISION VALUES
        // ======================================================

        boolean hasTarget = vision.hasTarget();

        double rawYaw = -vision.getTargetYaw().getDegrees();  // degrees
        double rawLat = vision.getTargetLateralOffset();       // meters
        double rawFwd = vision.getTargetForwardDistance();     // meters

        // ======================================================
        // 2) LOW-PASS FILTER / SMOOTHING
        // ======================================================

        filteredYaw = 0.8 * filteredYaw + 0.2 * rawYaw;
        filteredLat = 0.8 * filteredLat + 0.2 * rawLat;
        filteredFwd = 0.8 * filteredFwd + 0.2 * rawFwd;

        double yawDeg = filteredYaw;   // degrees
        double latCam = filteredLat;   // meters
        double fwdCam = filteredFwd;   // meters

        // ------------------------------------------------------
        // 2.5) CAMERA-OFFSET ROTATION COMPENSATION
        // ------------------------------------------------------
        // Because the camera is left of robot center, the robot must rotate
        // slightly for its CENTERLINE to point at the tag when the camera sees
        // a centered tag. We compute that geometry here:
        //
        //   angle = atan(lateralOffset / (targetDist + forwardOffset))
        //
        // This angle is SUBTRACTED from yawDeg so rotation aligns ROBOT center.

        double cameraYawComp = Math.toDegrees(Math.atan2(
                kCameraLateralOffsetMeters,
                kDesiredFrontDistanceMeters + kCameraForwardOffsetMeters
        ));

        yawDeg -= (cameraYawComp * 0.1);

        double absYaw = Math.abs(yawDeg);

        // Update last time a tag was seen
        double now = Timer.getFPGATimestamp();
        if (hasTarget) {
            lastSeenTime = now;
        }

        boolean withinGrace = (now - lastSeenTime) < 0.30; // 200 ms

        // ======================================================
        // 3) ROTATION CONTROL (aim robot center at tag)
        // ======================================================

        double rotCmd = kRotP * yawDeg;

        if (absYaw < kYawDeadband) {
            rotCmd = 0;

        } else if (absYaw < kRotSoftZoneDeg) {
            rotCmd *= 0.3;  // soften near target
            if (Math.abs(rotCmd) < kMinRot)
                rotCmd = Math.copySign(kMinRot, rotCmd);

        } else {
            if (Math.abs(rotCmd) < kMinRot)
                rotCmd = Math.copySign(kMinRot, rotCmd);
        }

        rotCmd = MathUtil.clamp(rotCmd, -kMaxRot, kMaxRot);

        // ======================================================
        // 4) STRAFE CONTROL (left/right centering)
        // ======================================================

        double latDesiredCam = -kCameraLateralOffsetMeters; // where camera should be at alignment
        double latError = latCam - latDesiredCam;

        double strafeCmd = -kStrafeP * latError;

        if (Math.abs(latError) < kStrafeDeadband)
            strafeCmd = 0;

        strafeCmd = MathUtil.clamp(strafeCmd, -kMaxStrafe, kMaxStrafe);

        // ======================================================
        // 5) FORWARD CONTROL (distance to reef)
        // ======================================================

        double robotFrontDist = fwdCam - kCameraForwardOffsetMeters;

        double distError = robotFrontDist - kDesiredFrontDistanceMeters;
        if (distError < 0)
            distError = 0;

        double driveCmd = kDriveP * distError;

        if (distError < kDriveDeadband)
            driveCmd = 0;

        driveCmd = Math.min(kMaxDrive, driveCmd);

        // ======================================================
        // 6) LOST TARGET FALLBACK (short grace period)
        // ======================================================

        if (!hasTarget) {
            if (withinGrace) {
                // decay translation but preserve rotation direction
                driveCmd = lastDriveCmd * 0.8;
                strafeCmd = lastStrafeCmd * 0.8;
                rotCmd = lastRotCmd;
            } else {
                driveCmd = 0;
                strafeCmd = 0;
            }
        }

        // ======================================================
        // 7) APPLY SWERVE COMMANDS
        // ======================================================

        request
                .withVelocityX(strafeCmd) // left/right
                .withVelocityY(driveCmd)  // forward
                .withRotationalRate(rotCmd);

        drivetrain.setControl(request);

        // Save for fallback
        lastDriveCmd = driveCmd;
        lastStrafeCmd = strafeCmd;
        lastRotCmd = rotCmd;
    }

    // ----------------------------------------------------------
    // end()
    // ----------------------------------------------------------

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign] END (interrupted=" + interrupted + ")");
        drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}
