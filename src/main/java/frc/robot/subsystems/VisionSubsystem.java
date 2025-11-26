// ============================================================
//  VisionSubsystem.java (LIMELIGHT VERSION – drop-in replacement)
// ------------------------------------------------------------
//
//  This subsystem replaces PhotonVision with Limelight NT values.
//
//  It STILL exposes THREE critical measurements:
//
//    1) getTargetYaw()               → rotation error (deg)
//    2) getTargetLateralOffset()    → left/right offset (meters)
//    3) getTargetForwardDistance()  → forward distance (meters)
//
//  Limelight Notes:
//  - tx  = horizontal offset (deg)
//  - ty  = vertical offset (deg)
//  - tv  = 0/1 target validity
//  - botpose_wpiblue/red gives 3D robot pose; we do NOT need it
//    for VisionAlign, which only needs offset to the *tag*.
//  - camtran = camera-to-tag 3D transform (meters)
//
//  camtran format (6-element array):
//     [X, Y, Z, roll, pitch, yaw]
//     X = forward (m)
//     Y = left    (m)
//     Z = up      (m)
//
// ============================================================

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    // ------------------------------------------------------------
    // Limelight NT table
    // ------------------------------------------------------------
    private final NetworkTable limelight;

    // Cached NT values
    private double tv = 0;
    private double tx = 0;
    private double[] camtran = new double[6];

    public VisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // ------------------------------------------------------------
    // BASIC HAS-TARGET CHECK
    // ------------------------------------------------------------
    public boolean hasTarget() {
        return tv > 0.5;
    }

    // ------------------------------------------------------------
    // 1) ROTATION (Yaw), degrees
    // ------------------------------------------------------------
    public Rotation2d getTargetYaw() {
        // Limelight:
        // +tx = target is to the RIGHT
        // −tx = target is to the LEFT
        //
        // Our original PV version inverted or adjusted signs inside VisionAlign.
        // We DO NOT invert here — VisionAlign already handles sign.
        return Rotation2d.fromDegrees(tx);
    }

    // ------------------------------------------------------------
    // 2) LATERAL OFFSET (meters, left/right)
    // ------------------------------------------------------------
    public double getTargetLateralOffset() {
        if (!hasTarget())
            return 0.0;

        // camtran[1] = Y (left +, right −)
        double lateral = camtran[1];

        SmartDashboard.putNumber("Vision/LateralOffsetMeters", lateral);
        return lateral;
    }

    // ------------------------------------------------------------
    // 3) FORWARD DISTANCE (meters)
    // ------------------------------------------------------------
    public double getTargetForwardDistance() {
        if (!hasTarget())
            return 0.0;

        // camtran[0] = X (forward distance)
        double forward = camtran[0];

        SmartDashboard.putNumber("Vision/ForwardDistanceMeters", forward);
        return forward;
    }

    // ------------------------------------------------------------
    // PERIODIC — pull NT values once per loop
    // ------------------------------------------------------------
    @Override
    public void periodic() {

        tv = limelight.getEntry("tv").getDouble(0.0);
        tx = limelight.getEntry("tx").getDouble(0.0);
        camtran = limelight.getEntry("camtran").getDoubleArray(new double[6]);

        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/TargetYaw(deg)", getTargetYaw().getDegrees());
    }
}
