package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

/**
 * VisionAlign (Option B: proper)
 * - Auto-rotate to center tag (tx -> 0)
 * - Driver can translate (forward + strafe) via suppliers
 *
 * For "strafe only while aligning", bind vx supplier to () -> 0.0.
 * Robot-centric on purpose: simple + predictable for drivers.
 */
public class VisionAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;

    // Driver translation suppliers (robot-centric)
    private final DoubleSupplier vxMetersPerSec; // forward
    private final DoubleSupplier vyMetersPerSec; // strafe

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public VisionAlign(
            CommandSwerveDrivetrain drivetrain,
            Limelight4 limelight,
            DoubleSupplier vxMetersPerSec,
            DoubleSupplier vyMetersPerSec) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.vxMetersPerSec = vxMetersPerSec;
        this.vyMetersPerSec = vyMetersPerSec;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("LL4/alignActive", true);
    }

    @Override
    public void execute() {
        // Safety: if no target, stop (and do NOT drift)
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

        // Driver translation (robot-centric)
        double vx = vxMetersPerSec.getAsDouble();
        double vy = vyMetersPerSec.getAsDouble();

        drivetrain.setControl(
                request.withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omegaRadPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("LL4/alignActive", false);
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }
}
