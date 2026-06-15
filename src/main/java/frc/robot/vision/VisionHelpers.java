package frc.robot.vision;

public class VisionHelpers {

    // Degrees offset for left/right reef posts relative to tag center
    public static final double CENTER_OFFSET_DEG = 0.0;
    public static final double LEFT_POST_OFFSET_DEG  = -8.0;
    public static final double RIGHT_POST_OFFSET_DEG = +8.0;

    // PID constants (simple P-control)
    public static final double kP_TURN = 0.03;     // rotation
    public static final double kP_STRAFE = 0.02;   // optional later
    public static final double FORWARD_SPEED = -0.15; // constant walk-in
}
