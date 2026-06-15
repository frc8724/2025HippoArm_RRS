package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;

/**
 * VisionAlignShoot
 * ----------------
 * Centers the ROBOT (shooter line) on the CENTER of the reef segment
 * and faces the reef. No forward movement, no arm control.
 *
 * This uses Limelight data via VisionSubsystem:
 *  - getTargetYaw()              → yaw error
 *  - getTargetLateralOffset()    → camera Y in meters
 *
 * Camera is offset laterally from the shooter, so we account for that
 * so that the SHOOTER (robot centerline) is aligned, not the camera.
 *
 * 3-second timeout included.
 */
public class VisionAlignShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // Camera offsets
    private static final double kCameraLateralOffsetMeters = 0.08573; // left of center

    // Rotation control
    private static final double kRotP = 0.035;
    private static final double kMaxRot = 2.0;
    private static final double kYawDeadband = 0.7; // tighter than VisionAlign

    // Strafe control
    private static final double kStrafeP = 3.0;
    private static final double kStrafeDeadband = 0.015; // tighter for shooter
    private static final double kMaxStrafe = 1.0;

    // Timeout (seconds)
    private static final double kTimeoutSec = 3.0;

    private final Timer timer = new Timer();

    private boolean aligned = false;
    private boolean lastHasTarget = false;
    private double lastYawDeg = 0.0;
    private double lastLatError = 0.0;

    public VisionAlignShoot(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlignShoot] Initialized");
        timer.reset();
        timer.start();
        aligned = false;
    }

    @Override
    public void execute() {

        boolean hasTarget = vision.hasTarget();

        // Same convention: negate tx to keep prior behavior
        double yawDeg = -vision.getTargetYaw().getDegrees();
        double latCam = vision.getTargetLateralOffset();

        System.out.println("[VisionAlignShoot] hasTarget=" + hasTarget +
                " yawDeg=" + yawDeg +
                " latCam=" + latCam);

        // -------------------------
        // Rotation control
        // -------------------------
        double rotCmd = kRotP * yawDeg;

        if (Math.abs(yawDeg) < kYawDeadband)
            rotCmd = 0.0;

        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));

        // -------------------------
        // Strafe control
        //
        // We want the SHOOTER (robot center) on the reef centerline.
        // Camera is left of center by kCameraLateralOffsetMeters, so:
        //
        // Desired camera Y when the shooter is centered is:
        //   latDesiredCam = -kCameraLateralOffsetMeters
        // -------------------------
        double latDesiredCam = -kCameraLateralOffsetMeters;
        double latError = latCam - latDesiredCam;

        double strafeCmd = kStrafeP * latError;

        if (Math.abs(latError) < kStrafeDeadband)
            strafeCmd = 0.0;

        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));

        // No forward drive for this command
        double driveCmd = 0.0;

        // If no target: hold still and wait (timeout will bail us out)
        if (!hasTarget) {
            strafeCmd = 0.0;
            rotCmd = 0.0;
        }

        // Side-facing axis mapping
        request = request
                .withVelocityX(strafeCmd)
                .withVelocityY(driveCmd)
                .withRotationalRate(rotCmd);

        drivetrain.setControl(request);

        // Store for finish logic
        lastHasTarget = hasTarget;
        lastYawDeg = yawDeg;
        lastLatError = latError;

        boolean yawAligned = Math.abs(yawDeg) < kYawDeadband;
        boolean latAligned = Math.abs(latError) < kStrafeDeadband;

        aligned = hasTarget && yawAligned && latAligned;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlignShoot] End, interrupted=" + interrupted);
        timer.stop();

        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Finish if aligned, or if timeout occurs (even if we never saw a target).
        if (aligned && lastHasTarget) {
            return true;
        }
        return timer.hasElapsed(kTimeoutSec);
    }
}
