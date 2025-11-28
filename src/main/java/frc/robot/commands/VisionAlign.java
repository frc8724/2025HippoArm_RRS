package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import frc.robot.commands.ReefTargetSide;




/**
 * VisionAlign (Side-Facing Scoring Version)
 * -----------------------------------------
 * Aligns the robot to a reef post (LEFT or RIGHT) when the robot
 * scores to its RIGHT side. Uses AprilTag camera-to-target transform
 * from VisionSubsystem (Limelight backend).
 *
 * Robot Coordinate Frame (WPILib / CTRE Standard):
 * +X = forward
 * +Y = left
 *
 * Your Mechanism Orientation:
 * Arm + camera face the RIGHT side of the robot.
 * → "Drive toward the reef" is robot -Y direction,
 * but we handle this with axis remap:
 *
 * CONTROL MAPPING:
 * - driveCmd (move toward reef)   → VelocityY
 * - strafeCmd (center left/right) → VelocityX
 * - rotCmd (face tag)             → RotationalRate
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final ReefTargetSide side;

    // Phoenix 6 swerve request
    private SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
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

    /** Reef post is 16.5 cm left/right from reef center. */
    private static final double kReefPostOffsetMeters = 0.165;

    // ----------------------------------------------------------
    // ROTATION CONTROL CONSTANTS
    // ----------------------------------------------------------

    private static final double kRotP = 0.030; // yaw P gain
    private static final double kMaxRot = 2.0; // rad/sec clamp
    private static final double kYawDeadband = 1.0; // degrees

    // ----------------------------------------------------------
    // STRAFING CONTROL CONSTANTS (left/right centering)
    // ----------------------------------------------------------

    private static final double kStrafeP = 2.5;
    private static final double kStrafeDeadband = 0.025; // meters
    private static final double kMaxStrafe = 1.0; // m/s

    // ----------------------------------------------------------
    // FORWARD DRIVE CONTROL CONSTANTS (toward reef)
    // ----------------------------------------------------------

    /** Robot-front target distance from reef base = 20 in ≈ 0.508 m */
    private static final double kDesiredFrontDistanceMeters = 0.508;

    private static final double kDriveP = 1.3;
    private static final double kDriveDeadband = 0.03; // m
    private static final double kMaxDrive = 0.90; // m/s

    // ----------------------------------------------------------
    // STATE FOR LOST-TARGET FALLBACK & FINISH LOGIC
    // ----------------------------------------------------------

    private double lastSeenTime = 0;
    private double lastDriveCmd = 0;
    private double lastStrafeCmd = 0;
    private double lastRotCmd = 0;

    private double lastFwdCmd = 0;
    private double lastLatCmd = 0;
    private double lastYawCmd = 0;

    private boolean lastHasTarget = false;
    private boolean aligned = false;

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, ReefTargetSide side) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.side = side;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized (robot-right scoring, side=" + side + ")");
        lastSeenTime = Timer.getFPGATimestamp();
        aligned = false;
    }

    @Override
    public void execute() {

        // ------------------------------------------------------
        // 1) READ ALL VISION DATA
        // ------------------------------------------------------

        boolean hasTarget = vision.hasTarget();

        // Limelight tx is positive to the RIGHT.
        // We negate here to keep the existing math convention.
        double yawDeg = -vision.getTargetYaw().getDegrees(); // rotational error

        double latCam = vision.getTargetLateralOffset();     // camera→tag Y (meters)
        double fwdCam = vision.getTargetForwardDistance();   // camera→tag X (meters)

        System.out.println("[VisionAlign] hasTarget=" + hasTarget +
                " yaw=" + yawDeg +
                " latCam=" + latCam +
                " fwdCam=" + fwdCam);

        double now = Timer.getFPGATimestamp();

        // ------------------------------------------------------
        // LOST-TARGET HANDLING (SAFE FORWARD FALLOFF)
        // ------------------------------------------------------

        if (hasTarget) {
            lastSeenTime = now;
            lastFwdCmd = lastDriveCmd;
            lastLatCmd = lastStrafeCmd;
            lastYawCmd = lastRotCmd;
        }

        double timeSinceSeen = now - lastSeenTime;
        boolean withinGrace = timeSinceSeen < 0.20; // 200ms

        // ------------------------------------------------------
        // 2) ROTATION CONTROL (face the AprilTag)
        // ------------------------------------------------------

        double rotCmd = kRotP * yawDeg;

        if (Math.abs(yawDeg) < kYawDeadband)
            rotCmd = 0;

        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));

        // ------------------------------------------------------
        // 3) STRAFE CONTROL (left/right centering)
        //
        // We want the SHOOTER (robot centerline) to line up with
        // either the LEFT or RIGHT post, with the camera offset
        // taken into account. Camera is left of center, so:
        //
        // - For CENTER: latDesiredCam = -kCameraLateralOffsetMeters
        // - For LEFT post:  CENTER + kReefPostOffsetMeters
        // - For RIGHT post: CENTER - kReefPostOffsetMeters
        // ------------------------------------------------------

        double latDesiredCamCenter = -kCameraLateralOffsetMeters;
        double latDesiredCam = latDesiredCamCenter;

        switch (side) {
            case LEFT:
                latDesiredCam = latDesiredCamCenter + kReefPostOffsetMeters;
                break;
            case RIGHT:
                latDesiredCam = latDesiredCamCenter - kReefPostOffsetMeters;
                break;
            default:
                break;
        }

        double latError = latCam - latDesiredCam;

        double strafeCmd = kStrafeP * latError;

        if (Math.abs(latError) < kStrafeDeadband)
            strafeCmd = 0;

        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));

        // ------------------------------------------------------
        // 4) FORWARD DRIVE (toward reef)
        // ------------------------------------------------------

        // Convert camera→tag distance into ROBOT-FRONT→reef distance
        double robotFrontDist = fwdCam - kCameraForwardOffsetMeters;

        double distError = robotFrontDist - kDesiredFrontDistanceMeters;

        // Don't overshoot closer than desired
        if (distError < 0)
            distError = 0;

        double driveCmd = kDriveP * distError;

        if (distError < kDriveDeadband)
            driveCmd = 0;

        driveCmd = Math.min(kMaxDrive, driveCmd);

        // ------------------------------------------------------
        // LOST-TARGET FALLBACK APPLY
        // ------------------------------------------------------

        if (!hasTarget) {
            if (withinGrace) {
                double decay = MathUtil.clamp(fwdCam / 0.8, 0.0, 1.0);
                driveCmd = lastFwdCmd * decay;
                strafeCmd = lastLatCmd;
                rotCmd = lastYawCmd;
            } else {
                driveCmd = 0.0;
                strafeCmd = 0.0;
                // keep rotCmd to allow searching
            }
        }

        // ------------------------------------------------------
        // 5) CONTROL AXIS ROTATION (SIDE-FACING)
        //
        // driveCmd moves robot toward reef
        // strafeCmd centers the robot (left/right)
        // ------------------------------------------------------

        request = request
                .withVelocityX(strafeCmd)
                .withVelocityY(driveCmd)
                .withRotationalRate(rotCmd);

        drivetrain.setControl(request);

        lastDriveCmd = driveCmd;
        lastStrafeCmd = strafeCmd;
        lastRotCmd = rotCmd;

        // ------------------------------------------------------
        // 6) FINISH CRITERIA
        // ------------------------------------------------------
        boolean yawAligned = Math.abs(yawDeg) < kYawDeadband;
        boolean latAligned = Math.abs(latError) < kStrafeDeadband;
        boolean distAligned = Math.abs(distError) < kDriveDeadband;

        aligned = hasTarget && yawAligned && latAligned && distAligned;
        lastHasTarget = hasTarget;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign] End, interrupted=" + interrupted);

        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Auto-finish when aligned; can also be interrupted when the driver
        // releases the trigger (when bound with .whileTrue()).
        return aligned;
    }
}
