package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import static frc.robot.RobotContainer.*;


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

    /** Front-of-robot target distance from tag (0.35 m ≈ 13.8 in) */
    private static final double kDesiredFrontDistanceMeters = 0.35;

    private static final double kDriveP = 1.3;
    private static final double kDriveDeadband = 0.03; // m
    private static final double kMaxDrive = 0.90; // m/s

    // ----------------------------------------------------------
    // STATE FOR LOST-TARGET FALLBACK
    // ----------------------------------------------------------

    private double lastSeenTime = 0;
    private double lastDriveCmd = 0;
    private double lastStrafeCmd = 0;
    private double lastRotCmd = 0;

    private double lastFwdCmd = 0;
    private double lastLatCmd = 0;
    private double lastYawCmd = 0;

    // ----------------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------------

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized (robot-right scoring)");
        lastSeenTime = Timer.getFPGATimestamp();
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

        double now = Timer.getFPGATimestamp();

        // ------------------------------------------------------
        // LOST-TARGET HANDLING (SAFE FORWARD FALLOFF)
        // Allows brief continuation (≤200ms), decay by distance,
        // prevents blind charging, driver is the safety interlock.
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
        // ------------------------------------------------------

        double latDesiredCam = -kCameraLateralOffsetMeters;
        double latError = latCam - latDesiredCam;

        double strafeCmd = kStrafeP * latError;

        if (Math.abs(latError) < kStrafeDeadband)
            strafeCmd = 0;

        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));

        // ------------------------------------------------------
        // 4) FORWARD DRIVE (toward reef)
        // ------------------------------------------------------

        double robotFrontDist = fwdCam - kCameraForwardOffsetMeters;

        double distError = robotFrontDist - kDesiredFrontDistanceMeters;

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
                // keep rotCmd to search
            }
        }

        // ------------------------------------------------------
        // 5) CONTROL AXIS ROTATION (CRITICAL FOR YOUR ROBOT)
        //
        // driveCmd moves robot toward reef
        // strafeCmd centers the robot
        //
        // ------------------------------------------------------

        request = request
                .withVelocityX(strafeCmd)
                .withVelocityY(driveCmd)
                .withRotationalRate(rotCmd);

        drivetrain.setControl(request);

        lastDriveCmd = driveCmd;
        lastStrafeCmd = strafeCmd;
        lastRotCmd = rotCmd;
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
