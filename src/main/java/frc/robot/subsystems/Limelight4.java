package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.VisionConstants;

public class Limelight4 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public Limelight4(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Create the checkbox on Shuffleboard. Default OFF.
        SmartDashboard.putBoolean(VisionConstants.FUSE_ENABLE_KEY, false);

        // Optional: publish these once so widgets are easy to add even before a tag is
        // seen
        SmartDashboard.putBoolean("LL4/usingMeasurement", false);
        SmartDashboard.putBoolean("LL4/hasTarget", false);
        SmartDashboard.putNumber("LL4/tagCount", 0);
        SmartDashboard.putNumber("LL4/poseX", 0.0);
        SmartDashboard.putNumber("LL4/poseY", 0.0);
        SmartDashboard.putNumber("LL4/tx", 0.0);
        SmartDashboard.putNumber("LL4/ty", 0.0);
    }

    @Override
    public void periodic() {
        // Read fusion enable from Shuffleboard (runtime toggle)
        boolean fuseEnabled = SmartDashboard.getBoolean(VisionConstants.FUSE_ENABLE_KEY, false);

        // 1) Get current drive state (pose + speeds)
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();

        // WPILib chassis speeds are rad/sec. Limelight wants deg/sec for yaw rate hint.
        double yawRateDegPerSec = Units.radiansToDegrees(driveState.Speeds.omegaRadiansPerSecond);

        // 2) Provide robot orientation hint for MegaTag2
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME,
                headingDeg,
                yawRateDegPerSec,
                0, 0, 0, 0);

        // 3) Get a pose estimate (MegaTag2, WPILib blue coords)
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);

        // Always publish these (stable Shuffleboard widgets)
        SmartDashboard.putBoolean("LL4/usingMeasurement", false);
        SmartDashboard.putNumber("LL4/tx", LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("LL4/ty", LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME));

        if (llMeasurement == null) {
            // No measurement this loop; still publish a reasonable hasTarget state from tv
            SmartDashboard.putBoolean("LL4/hasTarget", LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME));
            SmartDashboard.putNumber("LL4/tagCount", 0);
            return;
        }

        // Publish pose/debug values regardless of whether we fuse
        SmartDashboard.putNumber("LL4/tagCount", llMeasurement.tagCount);
        SmartDashboard.putNumber("LL4/poseX", llMeasurement.pose.getX());
        SmartDashboard.putNumber("LL4/poseY", llMeasurement.pose.getY());
        SmartDashboard.putBoolean("LL4/hasTarget", llMeasurement.tagCount > 0);

        // 4) Gate fusion:
        // - must be enabled from dashboard
        // - must see at least one tag
        // - don't trust while spinning too fast
        if (fuseEnabled
                && llMeasurement.tagCount > 0
                && Math.abs(driveState.Speeds.omegaRadiansPerSecond) < VisionConstants.MAX_VISION_OMEGA_RAD_PER_SEC)

            SmartDashboard.putBoolean("LL4/usingMeasurement", true);

        drivetrain.addVisionMeasurement(
                llMeasurement.pose,
                llMeasurement.timestampSeconds);

        boolean visionTrusted = fuseEnabled &&
                llMeasurement != null &&
                llMeasurement.tagCount > 0 &&
                Math.abs(driveState.Speeds.omegaRadiansPerSecond) < VisionConstants.MAX_VISION_OMEGA_RAD_PER_SEC;

        SmartDashboard.putBoolean("LL4/visionTrusted", visionTrusted);

    }

    // === Helper methods for commands ===

    public boolean hasTarget() {
        return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
    }

    public double getTxDegrees() {
        return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
    }

    public double getTyDegrees() {
        return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
    }

    // === LED + Pipeline control helpers ===

    public void setLEDOn() {
        LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME);
    }

    public void setLEDOff() {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
    }

    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME, index);
    }
}
