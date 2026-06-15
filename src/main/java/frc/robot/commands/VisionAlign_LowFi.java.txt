package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.commands.ReefTargetSide;




/**
 * VisionAlign_LowFi
 * -----------------
 * A TEMPORARY alignment command for Limelight 2+ that does NOT
 * require 3D transforms. Uses ONLY tx (horizontal offset)
 * to align the robot to the left or right reef post.
 *
 * NO forward driving. You drive forward manually.
 *
 * Behaviors:
 *  - Rotate robot until AprilTag is straight (tx ~ 0)
 *  - Strafe robot left/right to account for camera offset and chosen post
 *
 * This is ONLY for LL2+ testing and driver practice.
 */
public class VisionAlign_LowFi extends Command {

    // Add at top of class:
    private int lockedTagID = -1;
    private boolean tagIsLocked = false;

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final ReefTargetSide side;

    // Swerve command request
    private SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // Camera → robot geometry
    private static final double kCameraLateralOffsetMeters = 0.08573;   // 3.375"
    private static final double kReefPostOffsetMeters = 0.165;         // 16.5 cm post spacing

    // Gains
    private static final double kRotP = 0.025;
    private static final double kStrafeP = 0.75;

    private static final double kMaxRot = 2.0;
    private static final double kMaxStrafe = 0.8;

    public VisionAlign_LowFi(CommandSwerveDrivetrain drivetrain,
                             VisionSubsystem vision,
                             ReefTargetSide side) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.side = side;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign_LowFi] Init (side=" + side + ")");
    }

    @Override
    public void execute() {
    
        boolean hasTarget = vision.hasTarget();
        int tid = vision.getTargetID();   // <-- NEW (must add to VisionSubsystem)
    
        // ------------- TAG LOCK LOGIC -----------------
        if (hasTarget && !tagIsLocked) {
            // First frame that sees a target → lock it
            lockedTagID = tid;
            tagIsLocked = true;
            System.out.println("[LowFi] Locked to Tag ID = " + lockedTagID);
        }
    
        // If no target OR wrong tag → hold still
        if (!hasTarget || tid != lockedTagID) {
            drivetrain.setControl(
                request.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            return;
        }
    
        // ------------- NORMAL LOW-FI MATH --------------
    
        double tx = vision.getTargetYaw().getDegrees();
    
        // Turn toward tag
        double rotCmd = -kRotP * tx;
        rotCmd = Math.max(-kMaxRot, Math.min(kMaxRot, rotCmd));
    
        // Compute desired lateral offset
        double desiredLat = -kCameraLateralOffsetMeters;
    
        switch (side) {
            case LEFT:
                desiredLat += kReefPostOffsetMeters;
                break;
            case RIGHT:
                desiredLat -= kReefPostOffsetMeters;
                break;
        }
    
        // Fake lateral estimate using tx
        double latError = (-tx / 25.0) + desiredLat;
    
        double strafeCmd = kStrafeP * latError;
        strafeCmd = Math.max(-kMaxStrafe, Math.min(kMaxStrafe, strafeCmd));
    
        request = request
                .withVelocityX(strafeCmd)
                .withVelocityY(0)
                .withRotationalRate(rotCmd);
    
        drivetrain.setControl(request);
    }
        
    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign_LowFi] End");
        drivetrain.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false; // driver holds trigger
    }
}
