package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * VisionAlign
 * ------------
 * Strafes and rotates the robot to face and center on a detected vision target.
 * Compatible with Phoenix 6 free tier.
 */
public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drive;
    private final VisionSubsystem vision;

    // Tuneable proportional constants
    private static final double kRot = 0.02; // rotation gain
    private static final double kStrafe = 0.012; // strafe gain

    // Tolerances to prevent oscillation
    private static final double yawDeadbandDeg = 1.5;
    private static final double areaTarget = 8.0; // optional, for distance later

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.drive = drivetrain;
        this.vision = visionSubsystem;
        addRequirements(drivetrain, visionSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign] Initialized (strafe + rotate)");

        // TEMPORARY: Force drivetrain motion to verify control path works
        drive.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0.4)
                        .withRotationalRate(0));
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // Stop if no target
            drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double yawDeg = vision.getTargetYaw().getDegrees();
        double rotCmd = 0;
        double strafeCmd = 0;

        // --- Rotation correction ---
        if (Math.abs(yawDeg) > yawDeadbandDeg) {
            rotCmd = -kRot * yawDeg; // negative to rotate toward target
        }

        // --- Strafe correction ---
        // PhotonVision's +Yaw means target is left → move left (negative Y)
        strafeCmd = kStrafe * yawDeg;

        // Limit speeds to safe range (-1 to 1 m/s equivalent)
        rotCmd = Math.max(Math.min(rotCmd, 0.5), -0.5);
        strafeCmd = Math.max(Math.min(strafeCmd, 0.6), -0.6);

        // Apply the motion
        drive.setControl(
                driveRequest
                        .withVelocityX(0) // forward speed (we'll add this later)
                        .withVelocityY(strafeCmd) // sideways motion
                        .withRotationalRate(rotCmd) // rotation correction
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(new ChassisSpeeds(0, 0, 0)));
        System.out.println("[VisionAlign] Ended" + (interrupted ? " (interrupted)" : ""));
    }

    @Override
    public boolean isFinished() {
        return false; // runs while held
    }
}
