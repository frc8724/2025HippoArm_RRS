// ============================================================
//  VisionSubsystem.java  (Limelight 2024/2025 Compatible)
// ------------------------------------------------------------
//
//  This subsystem reads AprilTag data from Limelight using
//  targetpose_cameraspace (the modern replacement for camtran).
//
//  It exposes three critical measurements:
//
//    1) getTargetYaw()               → rotation error (deg)
//    2) getTargetLateralOffset()    → left/right distance (meters)
//    3) getTargetForwardDistance()  → forward distance (meters)
//
//  This supports our VisionAlign system where the robot scores
//  to its RIGHT SIDE.
//
//  Limelight targetpose_cameraspace format:
//    [ X, Y, Z, roll, pitch, yaw ]
//       X = forward (meters)
//       Y = left   (meters)
//       Z = up     (meters)
//
// ============================================================

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {

    // ------------------------------------------------------------
    // NetworkTables: Get the "limelight" table
    // ------------------------------------------------------------
    private final NetworkTable limelight =
            NetworkTableInstance.getDefault().getTable("limelight");

    // Cached pipeline values
    private boolean hasTarget = false;
    private double tx = 0.0;
    private double[] camSpace = new double[] {0, 0, 0, 0, 0, 0};

    public VisionSubsystem() {
        System.out.println("[VisionSubsystem] Initialized (Limelight targetpose_cameraspace)");
    }

    // ------------------------------------------------------------
    // Basic "has target" check
    // ------------------------------------------------------------
    public boolean hasTarget() {
        return hasTarget;
    }

    // ------------------------------------------------------------
    // 1) ROTATION — Yaw offset from tag (degrees)
    // ------------------------------------------------------------
    public Rotation2d getTargetYaw() {
        // Limelight tx: +tx = target left, -tx = target right
        // We want: +yaw = robot must turn CCW
        return Rotation2d.fromDegrees(-tx);
    }

    // ------------------------------------------------------------
    // 2) LATERAL OFFSET (meters)
    // ------------------------------------------------------------
    public double getTargetLateralOffset() {

        if (!hasTarget) return 0.0;

        double y = camSpace[1]; // LL cameraspace +Y = left of camera

        SmartDashboard.putNumber("Vision/LateralOffsetMeters", y);
        return y;
    }

    // ------------------------------------------------------------
    // 3) FORWARD DISTANCE (meters)
    // ------------------------------------------------------------
    public double getTargetForwardDistance() {

        if (!hasTarget) return 0.0;

        double x = camSpace[0]; // LL cameraspace +X = in front of camera

        SmartDashboard.putNumber("Vision/ForwardDistanceMeters", x);
        return x;
    }

    // ------------------------------------------------------------
    // Periodic update from Limelight
    // ------------------------------------------------------------
    @Override
    public void periodic() {

        // 1) hasTarget = (tv == 1)
        hasTarget = limelight.getEntry("tv").getDouble(0) > 0.5;

        // 2) read horizontal angle
        tx = limelight.getEntry("tx").getDouble(0.0);

        // 3) new LL 2024+ 3D camera-space pose
        camSpace = limelight
                .getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[] {0,0,0,0,0,0});

        // Debug outputs
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/tx", tx);

        SmartDashboard.putNumber("LL/Cam_X", camSpace.length > 0 ? camSpace[0] : 0);
        SmartDashboard.putNumber("LL/Cam_Y", camSpace.length > 0 ? camSpace[1] : 0);
        SmartDashboard.putNumber("LL/Cam_Z", camSpace.length > 0 ? camSpace[2] : 0);
        SmartDashboard.putNumber("LL/Cam_Length", camSpace.length);
    }
}
