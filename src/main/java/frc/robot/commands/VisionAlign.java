package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * VisionAlign (Side-Facing Scoring Version)
 * -----------------------------------------
 * This aligns the robot to an AprilTag when the ROBOT SCORES to its RIGHT side.
 *
 * Robot Coordinate Frame (WPILib / CTRE Standard):
 * +X = forward
 * +Y = left
 *
 * Your Mechanism Orientation:
 * Arm + camera face the RIGHT side of the robot.
 * → Therefore, "drive toward the reef" is robot -Y direction.
 * BUT since we use field-centric, robot Y is mapped to FIELD directions.
 *
 * CONTROL MAPPING FOR THIS ROBOT:
 * - driveCmd (move toward tag) → VelocityY
 * - strafeCmd (center left/right) → VelocityX
 * - rotCmd (face tag) → RotationalRate
 *
 * This is a simple +90° rotation of the control axes to match your mechanism.
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Phoenix 6 swerve request (immutable → rebuilt every loop)
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // ----------------------------------------------------------
    // CAMERA MOUNT OFFSETS (meters)
    // ----------------------------------------------------------

    /** Camera is 17.75 in behind robot front = 0.45085 m */
    private static final double kCameraForwardOffsetMeters = 0.45085;

    /** Camera is 3.375 in LEFT of robot centerline = 0.08573 m */
    private static final double kCameraLateralOffsetMeters = 0.08573;

    // ----------------------------------------------------------
    // ROTATION CONTROL CONSTANTS
    // ----------------------------------------------------------

    private static final double kRotP = 0.02; // yaw P gain
    private static final double kMaxRot = 1.5; // rad/sec clamp
    private static final double kYawDeadband = 1.5; // degrees

    // ----------------------------------------------------------
    // STRAFING CONTROL CONSTANTS (left/right centering)
    // ----------------------------------------------------------

    private static final double kStrafeP = 2.0;
    private static final double kStrafeDeadband = 0.02; // meters
    private static final double kMaxStrafe = 0.75; // m/s

    // ----------------------------------------------------------
    // FORWARD DRIVE CONTROL CONSTANTS (toward reef)
    // ----------------------------------------------------------

    /** Front-of-robot target distance from tag (0.35 m ≈ 13.8 in) */
    private static final double kDesiredFrontDistanceMeters = 0.35;

    private static final double kDriveP = 1.5;
    private static final double kDriveDeadband = 0.03; // m
    private static final double kMaxDrive = 1.0; // m/s

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // required so this command interrupts TeleopDrive
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized (robot-right scoring)");
    }

    @Override
    public void execute() {

        // ------------------------------------------------------
        // 1) READ ALL VISION DATA
        // ------------------------------------------------------

        boolean hasTarget = vision.hasTarget();

        double yawDeg = -vision.getTargetYaw().getDegrees(); // rotational error
        double latCam = vision.getTargetLateralOffset(); // camera→tag Y (meters)
        double fwdCam = vision.getTargetForwardDistance(); // camera→tag X (meters)

        System.out.println("[VisionAlign] hasTarget=" + hasTarget +
                " yaw=" + yawDeg +
                " latCam=" + latCam +
                " fwdCam=" + fwdCam);

        if (!hasTarget) {
            // If tag lost → STOP (never dead-reckon forward)
            request = request
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
            drivetrain.setControl(request);
            return;
        }

        // ------------------------------------------------------
        // 2) ROTATION CONTROL (face the AprilTag)
        // ------------------------------------------------------

        double rotCmd = kRotP * yawDeg;

        if (Math.abs(yawDeg) < kYawDeadband)
            rotCmd = 0;

        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));

        // ------------------------------------------------------
        // 3) STRAFE CONTROL (left/right centering, camera offset aware)
        // ------------------------------------------------------

        // When arm/centerline is aligned, camera sees tag at -offset
        double latDesiredCam = -kCameraLateralOffsetMeters;
        double latError = latCam - latDesiredCam;

        double strafeCmd = kStrafeP * latError;

        if (Math.abs(latError) < kStrafeDeadband)
            strafeCmd = 0;

        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));

        // ------------------------------------------------------
        // 4) FORWARD DRIVE CONTROL (toward reef)
        // robotFrontDist = distance from ROBOT FRONT → tag
        // ------------------------------------------------------

        double robotFrontDist = fwdCam - kCameraForwardOffsetMeters;

        double distError = robotFrontDist - kDesiredFrontDistanceMeters;

        // Never drive backward when too close
        if (distError < 0)
            distError = 0;

        double driveCmd = kDriveP * distError;

        if (distError < kDriveDeadband)
            driveCmd = 0;

        driveCmd = Math.min(kMaxDrive, driveCmd);

        // ------------------------------------------------------
        // 5) CONTROL AXIS ROTATION (CRITICAL FOR YOUR ROBOT)
        // ------------------------------------------------------
        //
        // Your arm faces robot-RIGHT, so:
        //
        // Robot-RIGHT (scoring direction) → +VelocityY
        // Robot-FORWARD → irrelevant for reef scoring
        // Robot-LEFT/RIGHT centering → +VelocityX
        //
        // This swaps the axes so that:
        //
        // driveCmd moves robot toward reef
        // strafeCmd centers the robot
        //
        // ------------------------------------------------------

        request = request
                .withVelocityX(strafeCmd) // left/right centering
                .withVelocityY(driveCmd) // toward reef (robot-right)
                .withRotationalRate(rotCmd); // face tag

        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign] End");

        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}
