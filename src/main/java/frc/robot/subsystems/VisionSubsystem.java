// ============================================================
//  VisionSubsystem.java
// ------------------------------------------------------------
//  Handles all vision data and provides yaw offset for alignment
//  commands using PhotonVision-compatible cameras (Limelight,
//  Arducam, etc.).
//
//  Includes "smoothed yaw hold" logic to prevent drivetrain
//  twitching when the camera momentarily loses the AprilTag due
//  to motion blur or vibration.
//
//  Compatible with: PhotonVision 2024+
//  Hardware tested: Limelight 2+ (rolling shutter)
// ============================================================

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    // ------------------------------------------------------------
    // Camera setup
    // ------------------------------------------------------------

    /**
     * PhotonVision camera instance. The name must match the
     * camera name configured in the PhotonVision dashboard.
     */
    private final PhotonCamera photonCamera;

    /** Stores the most recent vision processing result. */
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    // ------------------------------------------------------------
    // Yaw smoothing variables
    // ------------------------------------------------------------

    /** The last known yaw angle (in degrees) from a valid target. */
    private double lastValidYawDeg = 0.0;

    /** The timestamp of the most recent valid detection. */
    private double lastValidTime = 0.0;

    /**
     * Duration (in seconds) to hold the last known yaw value after
     * target loss before resetting to zero.
     */
    private static final double kYawHoldDuration = 0.25;

    // ------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------

    public VisionSubsystem() {
        // Replace "photonhippo" with your actual camera name as
        // defined in the PhotonVision interface.
        photonCamera = new PhotonCamera("Limelight");
    }

    // ------------------------------------------------------------
    // Core accessors
    // ------------------------------------------------------------

    /** Checks whether the camera currently sees any valid targets. */
    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    /**
     * Returns the yaw (horizontal offset) to the best target as a Rotation2d.
     * In PhotonVision, +Yaw means the target is LEFT of center; use the
     * negative sign to match standard field-oriented conventions if needed.
     */
    public Rotation2d getTargetYaw() {
        if (latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            return Rotation2d.fromDegrees(-target.getYaw());
        } else {
            return new Rotation2d(); // 0° if no target detected
        }
    }

    // ------------------------------------------------------------
    // Smoothed yaw method
    // ------------------------------------------------------------

    /**
     * Returns the target yaw as a Rotation2d, applying short-term smoothing.
     * If the target is momentarily lost (for example, while the robot moves),
     * this method holds the last known yaw value for up to kYawHoldDuration
     * seconds to prevent drivetrain twitching.
     *
     * This behavior gives the driver and alignment commands a steadier signal
     * and avoids jerky transitions between "target found" and "no target."
     */
    public Rotation2d getSmoothedTargetYaw() {

        // Get current timestamp
        double currentTime = Timer.getFPGATimestamp();

        // Fetch the most recent result directly from the camera
        PhotonPipelineResult result = photonCamera.getLatestResult();

        if (result.hasTargets()) {
            // Valid target detected
            PhotonTrackedTarget target = result.getBestTarget();
            lastValidYawDeg = target.getYaw(); // store yaw in degrees
            lastValidTime = currentTime; // store timestamp
            SmartDashboard.putBoolean("Vision/HasTarget", true);
            SmartDashboard.putBoolean("Vision/HoldingYaw", false);
            return Rotation2d.fromDegrees(-lastValidYawDeg);

        } else {
            // No target currently visible
            SmartDashboard.putBoolean("Vision/HasTarget", false);

            // If within the hold duration, continue returning last yaw
            if (currentTime - lastValidTime < kYawHoldDuration) {
                SmartDashboard.putBoolean("Vision/HoldingYaw", true);
                return Rotation2d.fromDegrees(-lastValidYawDeg);

            } else {
                // Hold period expired — return zero yaw
                SmartDashboard.putBoolean("Vision/HoldingYaw", false);
                return new Rotation2d();
            }
        }
    }

    // ------------------------------------------------------------
    // Periodic update loop
    // ------------------------------------------------------------

    /**
     * Called once per scheduler cycle (typically every 20 ms).
     * Updates the cached pipeline result and publishes telemetry
     * to SmartDashboard for debugging and driver feedback.
     */
    @Override
    public void periodic() {
        // Refresh the latest vision result from PhotonVision
        latestResult = photonCamera.getLatestResult();

        // Publish key diagnostic data
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Target Yaw (deg)", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Vision/Target Area",
                latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : 0.0);
    }
}
