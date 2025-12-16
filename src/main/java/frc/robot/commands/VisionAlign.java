package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

/**
 * VisionAlign (LL2/PhotonVision-style):
 * - Auto-rotate to center tag (tx -> 0)
 * - Auto-drive forward to a stop distance using ty-based distance estimate
 * - Driver can still strafe (vy) while held
 *
 * Robot-centric on purpose: simple + predictable for drivers.
 */
public class VisionAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;
    private final DoubleSupplier vyMetersPerSec; // driver strafe

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public VisionAlign(
            CommandSwerveDrivetrain drivetrain,
            Limelight4 limelight,
            DoubleSupplier vyMetersPerSec) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.vyMetersPerSec = vyMetersPerSec;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Safety: if no target, stop
        if (!limelight.hasTarget()) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // --- Rotation control (tx -> 0), using your proven sign convention ---
        double txDeg = limelight.getTxDegrees();
        double omegaRadPerSec = 0.0;

        if (Math.abs(txDeg) > VisionConstants.ROTATE_DEADBAND_DEG) {
            omegaRadPerSec = MathUtil.clamp(
                    -VisionConstants.ROTATE_KP_RAD_PER_SEC_PER_DEG * txDeg,
                    -VisionConstants.ROTATE_MAX_OMEGA_RAD_PER_SEC,
                    VisionConstants.ROTATE_MAX_OMEGA_RAD_PER_SEC);
        }

        // --- Forward control using ty-based distance estimate ---
        double tyDeg = limelight.getTyDegrees();
        double distanceM = estimateDistanceMetersFromTy(tyDeg);

        double forwardMps = 0.0;
        double errorM = distanceM - VisionConstants.ALIGN_STOP_DISTANCE_M;

        if (Math.abs(errorM) > VisionConstants.ALIGN_DISTANCE_DEADBAND_M) {
            forwardMps = MathUtil.clamp(
                    VisionConstants.ALIGN_FORWARD_KP * errorM,
                    -VisionConstants.ALIGN_MAX_FWD_MPS,
                    VisionConstants.ALIGN_MAX_FWD_MPS);
        }

        // NOTE: +X should be "forward". If your robot drives the wrong way, flip
        // forwardMps sign here.
        drivetrain.setControl(
                request.withVelocityX(forwardMps)
                        .withVelocityY(vyMetersPerSec.getAsDouble())
                        .withRotationalRate(omegaRadPerSec));
    }

    /**
     * Estimate distance to the target using camera pitch + ty.
     * distance = (targetHeight - cameraHeight) / tan(cameraPitch + ty)
     */
    private static double estimateDistanceMetersFromTy(double tyDeg) {
        double angleDeg = VisionConstants.CAMERA_PITCH_DEG + tyDeg;
        double angleRad = Units.degreesToRadians(angleDeg);

        // Avoid tan blowing up near +/- 90 degrees
        angleRad = MathUtil.clamp(angleRad, Units.degreesToRadians(-89), Units.degreesToRadians(89));

        double heightDiffM = VisionConstants.TAG_CENTER_HEIGHT_M - VisionConstants.CAMERA_HEIGHT_M;
        return heightDiffM / Math.tan(angleRad);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("LL4/alignActive", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("LL4/alignActive", false);
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }
}
