// ============================================================
//  VisionSubsystem.java  (FINAL — matches side-scoring robot)
// ------------------------------------------------------------
//
//  This subsystem reads AprilTag data from PhotonVision and
//  exposes THREE critical measurements:
//
//    1) getTargetYaw()               → rotation error (deg)
//    2) getTargetLateralOffset()    → left/right offset (meters)
//    3) getTargetForwardDistance()  → forward distance (meters)
//
//  These now support the full VisionAlign system where the
//  robot scores to ITS RIGHT SIDE.
//
//  Notes:
//  - PhotonVision 3D transform is used for translation.
//  - Yaw is read separately from the transform.
//  - latestResult is cached once per cycle (no double polling).
//
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
    // Camera instance (name MUST match PhotonVision)
    // ------------------------------------------------------------
    private final PhotonCamera photonCamera;

    /** Latest fetched result from PhotonVision (updated each periodic). */
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    // ------------------------------------------------------------
    // Optional yaw smoothing variables (currently unused by VisionAlign)
    // ------------------------------------------------------------
    private double lastValidYawDeg = 0.0;
    private double lastValidTime = 0.0;
    private static final double kYawHoldDuration = 0.25;

    // ------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------
    public VisionSubsystem() {
        photonCamera = new PhotonCamera("Limelight"); // YOUR camera name
    }

    // ------------------------------------------------------------
    // Basic "has target" check
    // ------------------------------------------------------------
    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    // ------------------------------------------------------------
    // 1) ROTATION — Yaw offset from tag
    // ------------------------------------------------------------
    public Rotation2d getTargetYaw() {
        if (!latestResult.hasTargets()) {
            return new Rotation2d(); // 0 degrees
        }

        PhotonTrackedTarget target = latestResult.getBestTarget();

        /**
         * PhotonVision:
         * +Yaw = tag LEFT of camera
         *
         * We NEGATE it so:
         * +Yaw = robot must rotate CCW (standard field conventions)
         */
        return Rotation2d.fromDegrees(target.getYaw());
    }

    // ------------------------------------------------------------
    // 2) LATERAL OFFSET (meters)
    // ------------------------------------------------------------
    public double getTargetLateralOffset() {
        if (!latestResult.hasTargets()) {
            SmartDashboard.putNumber("Vision/LateralOffsetMeters", 0.0);
            return 0.0;
        }

        PhotonTrackedTarget target = latestResult.getBestTarget();

        /**
         * PhotonVision 3D transform:
         * X = forward distance
         * Y = left/right
         * Z = vertical
         *
         * +Y = tag LEFT of camera
         * -Y = tag RIGHT of camera
         */
        double lateralMeters = target.getBestCameraToTarget().getY();

        SmartDashboard.putNumber("Vision/LateralOffsetMeters", lateralMeters);
        return lateralMeters;
    }

    // ------------------------------------------------------------
    // 3) FORWARD DISTANCE (meters)
    // ------------------------------------------------------------
    public double getTargetForwardDistance() {
        if (!latestResult.hasTargets()) {
            SmartDashboard.putNumber("Vision/ForwardDistanceMeters", 0.0);
            return 0.0;
        }

        PhotonTrackedTarget target = latestResult.getBestTarget();

        /**
         * PhotonVision:
         * +X = tag IN FRONT of camera
         * This is the raw forward distance from camera to tag.
         *
         * VisionAlign uses this but subtracts camera-forward-offset
         * to compute robot-front distance.
         */
        double forwardMeters = target.getBestCameraToTarget().getX();

        SmartDashboard.putNumber("Vision/ForwardDistanceMeters", forwardMeters);
        return forwardMeters;
    }

    // ------------------------------------------------------------
    // OPTIONAL: smoothed yaw (not used by our VisionAlign)
    // ------------------------------------------------------------
    public Rotation2d getSmoothedTargetYaw() {

        double currentTime = Timer.getFPGATimestamp();
        PhotonPipelineResult result = photonCamera.getLatestResult();

        if (result.hasTargets()) {
            lastValidYawDeg = result.getBestTarget().getYaw();
            lastValidTime = currentTime;
            SmartDashboard.putBoolean("Vision/HoldingYaw", false);
            return Rotation2d.fromDegrees(-lastValidYawDeg);
        }

        SmartDashboard.putBoolean("Vision/HoldingYaw", true);

        if (currentTime - lastValidTime < kYawHoldDuration) {
            return Rotation2d.fromDegrees(-lastValidYawDeg);
        }

        return new Rotation2d();
    }

    // ------------------------------------------------------------
    // Periodic (runs every ~20 ms)
    // ------------------------------------------------------------
    @Override
    public void periodic() {

        // Always update latestResult ONCE per loop
        latestResult = photonCamera.getLatestResult();

        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());

        // Quick convenience debug values
        SmartDashboard.putNumber("Vision/TargetYaw(deg)", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Vision/TargetArea",
                latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : 0.0);
    }
}
