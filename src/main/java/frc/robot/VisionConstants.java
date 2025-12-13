package frc.robot;

public final class VisionConstants {
    // Master switch: turn Limelight usage on/off
    public static final boolean kUseLimelight = false;

    // This must match the "Device Name" you set in the Limelight web UI
    public static final String LIMELIGHT_NAME = "limelight-front";

    // Don’t trust vision measurements if we're spinning faster than this
    public static final double MAX_VISION_OMEGA_RPS = 2.0;

    // Prevent instantiation
    private VisionConstants() {
    }
}
