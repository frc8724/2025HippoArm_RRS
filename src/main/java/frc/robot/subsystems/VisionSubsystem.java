// ============================================================
//  VisionSubsystem.java  — Limelight 2+ 3D AprilTag (Robot-Space Output)
// ------------------------------------------------------------
//
//  This subsystem does THREE things:
//
//    1) Reads targetpose_cameraspace from Limelight
//    2) Applies robot→camera transform (LL2+ has no Geometry UI)
//    3) Outputs ROBOT-SPACE translations for VisionAlign:
//
//       getTargetForwardDistance()  → +X (forward toward reef)
//       getTargetLateralOffset()    → +Y (left of robot center)
//       getTargetYaw()              → yaw error (deg)
//
//  Robot Frame (WPILib standard):
//    +X = forward
//    +Y = left
//    +Z = up
//
//  Limelight camera-space axes:
//    X+ = RIGHT of camera
//    Y+ = DOWN
//    Z+ = OUT of camera (forward)
// ============================================================

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.util.Units;

public class VisionSubsystem extends SubsystemBase {

    // ------------------------------------------------------------
    // TRANSFORM: Robot → Camera (YOU remounted LL to robot front)
    // ------------------------------------------------------------
    //
    // Frame: 22" square
    // Half-length = 11"
    // Pretend bumper = +3"
    // Camera is 14" ahead of robot center
    //
    // Height = 4.5"
    // Centered left/right
    //
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(14.0),   // forward from robot center
            Units.inchesToMeters(0.0),    // centered left-right
            Units.inchesToMeters(4.5)     // height
        ),
        new Rotation3d(0, 0, 0)           // LL faces forward, level
    );

    // Limelight table
    private final NetworkTable limelightTable;

    // Cached data each periodic()
    private boolean hasTarget = false;
    private double txDegrees = 0.0;
    private double[] targetPoseCam = new double[6]; // [X,Y,Z, roll,pitch,yaw]

    // Cached robot-space transform
    private Transform3d robotToTarget = new Transform3d();

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // ------------------------------------------------------------
    // Public getters
    // ------------------------------------------------------------

    public boolean hasTarget() {
        return hasTarget;
    }

    /** Returns LL tx as a Rotation2d */
    public Rotation2d getTargetYaw() {
        return Rotation2d.fromDegrees(txDegrees);
    }

    /** Robot-space lateral offset (+Y = left of robot) */
    public double getTargetLateralOffset() {
        double y = robotToTarget.getY();
        SmartDashboard.putNumber("Vision/RobotSpaceLateralY", y);
        return y;
    }

    /** Robot-space forward distance (+X = forward) */
    public double getTargetForwardDistance() {
        double x = robotToTarget.getX();
        SmartDashboard.putNumber("Vision/RobotSpaceForwardX", x);
        return x;
    }

    public int getTargetID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    // ------------------------------------------------------------
    // Periodic update
    // ------------------------------------------------------------
    @Override
    public void periodic() {

        // Basic state
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        hasTarget = tv > 0.5;

        txDegrees = limelightTable.getEntry("tx").getDouble(0.0);

        // Read the camera-space target transform
        targetPoseCam = limelightTable
                .getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[6]); // [X,Y,Z,roll,pitch,yaw]

        if (hasTarget && targetPoseCam.length >= 6) {

            // Build CAMERA→TARGET transform (meters + radians)
            Transform3d CAMERA_TO_TARGET = new Transform3d(
                new Translation3d(
                    targetPoseCam[0],                    // X+
                    -targetPoseCam[1],                   // Convert LL Y+ down → robot Z+, robot Y axis unaffected
                    targetPoseCam[2]                     // Z+
                ),
                new Rotation3d(
                    Math.toRadians(targetPoseCam[3]),
                    Math.toRadians(targetPoseCam[4]),
                    Math.toRadians(targetPoseCam[5])
                )
            );

            // Convert into ROBOT-SPACE
            robotToTarget = ROBOT_TO_CAMERA.plus(CAMERA_TO_TARGET);

        } else {
            robotToTarget = new Transform3d();
        }

        // Debug info
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/tx(deg)", txDegrees);
        SmartDashboard.putNumber("Vision/CamX", targetPoseCam.length > 0 ? targetPoseCam[0] : 0);
        SmartDashboard.putNumber("Vision/CamY", targetPoseCam.length > 1 ? targetPoseCam[1] : 0);
        SmartDashboard.putNumber("Vision/CamZ", targetPoseCam.length > 2 ? targetPoseCam[2] : 0);

        SmartDashboard.putNumber("Vision/RobotTargetX", robotToTarget.getX());
        SmartDashboard.putNumber("Vision/RobotTargetY", robotToTarget.getY());
    }
}
