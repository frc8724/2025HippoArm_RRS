package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Limelight4 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    boolean fuseEnabled = VisionConstants.kUseLimelight;

    public Limelight4(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Master switch: if false, do nothing
        // if (!VisionConstants.kUseLimelight) {
        // return;
        // }
        boolean fuseEnabled = VisionConstants.kUseLimelight;

        // 1. Get current drive state (pose + speeds) from CTRE drivetrain
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        // 2. Feed our gyro-based heading into Limelight's MegaTag2 helper
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME,
                headingDeg,
                0, 0, 0, 0, 0);

        // 3. Ask Limelight for a pose estimate in WPILib blue alliance coordinates
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                VisionConstants.LIMELIGHT_NAME);

        // If we didn't get a measurement, stop here
        if (llMeasurement == null) {
            return;
        }

        // Basic debug logging
        SmartDashboard.putBoolean("LL4/usingMeasurement", false);
        SmartDashboard.putNumber("LL4/tagCount", llMeasurement.tagCount);
        SmartDashboard.putNumber("LL4/poseX", llMeasurement.pose.getX());
        SmartDashboard.putNumber("LL4/poseY", llMeasurement.pose.getY());

        // 4. Basic gating:
        // - Must see at least one tag
        // - Don't trust vision when spinning too fast
        // if (llMeasurement.tagCount > 0
        // && Math.abs(omegaRps) < VisionConstants.MAX_VISION_OMEGA_RPS) {
        if (fuseEnabled
                && llMeasurement.tagCount > 0
                && Math.abs(omegaRps) < VisionConstants.MAX_VISION_OMEGA_RPS) {
            // Debug: show that we're using this measurement
            SmartDashboard.putBoolean("LL4/usingMeasurement", true);
            SmartDashboard.putNumber("LL4/tagCount", llMeasurement.tagCount);
            SmartDashboard.putNumber("LL4/poseX", llMeasurement.pose.getX());
            SmartDashboard.putNumber("LL4/poseY", llMeasurement.pose.getY());

            // 5. Feed the vision measurement into the swerve pose estimator
            drivetrain.addVisionMeasurement(
                    llMeasurement.pose,
                    llMeasurement.timestampSeconds);
        }
    }

    // === Helper methods for commands ===

    /** @return true if Limelight currently has any valid target. */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
    }

    /**
     * @return horizontal offset to the target in DEGREES (positive = target to the
     *         right).
     */
    public double getTxDegrees() {
        return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
    }

    /**
     * @return vertical offset to the target in DEGREES (positive = target above
     *         crosshair).
     */
    public double getTyDegrees() {
        return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
    }

    // === LED + Pipeline control helpers ===

    /** Force LEDs on. */
    public void setLEDOn() {
        LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME);
    }

    /** Force LEDs off. */
    public void setLEDOff() {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
    }

    /** Set the Limelight to a specific pipeline index (0–9). */
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME, index);
    }

}
