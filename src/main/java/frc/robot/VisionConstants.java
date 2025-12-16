package frc.robot;

public final class VisionConstants {
    // Master switch: turn Limelight usage on/off
    // public static final boolean kUseLimelight = false;

    // This must match the "Device Name" you set in the Limelight web UI
    public static final String LIMELIGHT_NAME = "limelight-front";

    // Don’t trust vision measurements if we're spinning faster than this (rad/sec)
    public static final double MAX_VISION_OMEGA_RAD_PER_SEC = 2.0;

    // Rotate-to-tag tuning
    public static final double ROTATE_KP_RAD_PER_SEC_PER_DEG = 0.06;
    public static final double ROTATE_MAX_OMEGA_RAD_PER_SEC = 2.0;
    public static final double ROTATE_DEADBAND_DEG = 1.0;

    // Start with Pose Fustion/PP
    public static final String FUSE_ENABLE_KEY = "LL4/FuseEnabled";

    // --- VisionAlign distance estimation (ty -> distance) ---
    // Measure these on the real robot!
    public static final double CAMERA_HEIGHT_M = 0.30; // camera lens height from floor
    public static final double CAMERA_PITCH_DEG = 20.0; // + up, - down
    public static final double TAG_CENTER_HEIGHT_M = 0.70; // AprilTag center height (game-specific)

    // How close we want to stop from the tag/reef
    public static final double ALIGN_STOP_DISTANCE_M = 0.60;

    // Forward control tuning
    public static final double ALIGN_FORWARD_KP = 0.8;
    public static final double ALIGN_MAX_FWD_MPS = 1.0;
    public static final double ALIGN_DISTANCE_DEADBAND_M = 0.05;

    // Prevent instantiation
    private VisionConstants() {
    }
}
