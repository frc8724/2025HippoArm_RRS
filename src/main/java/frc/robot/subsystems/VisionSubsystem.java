package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * VisionSubsystem
 * ----------------
 * Handles all vision data and provides yaw offset for alignment commands.
 * Designed for PhotonVision-compatible cameras (Limelight, Arducam, etc.)
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera photonCamera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    public VisionSubsystem() {
        // Replace "limelight" with your actual camera name from PhotonVision
        photonCamera = new PhotonCamera("photonhippo");
    }

    /** Checks whether the camera currently sees any targets. */
    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    /**
     * Returns the yaw (horizontal angle offset) to the best target.
     * If no target is visible, returns 0°.
     */
    public Rotation2d getTargetYaw() {
        if (latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            // In PhotonVision, +Yaw = target is left of center. Negate if needed.
            return Rotation2d.fromDegrees(-target.getYaw());
        } else {
            return new Rotation2d();
        }
    }

    @Override
    public void periodic() {
        // Update once per loop
        latestResult = photonCamera.getLatestResult();

        // Publish key data for debugging in Shuffleboard / SmartDashboard
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Target Yaw (deg)", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Vision/Target Area",
                latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : 0.0);
    }
}
