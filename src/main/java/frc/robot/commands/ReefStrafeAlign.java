package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

import java.util.function.DoubleSupplier;

public class ReefStrafeAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;

    // Driver forward/back (m/s). Strafe + rotation are vision-controlled.
    private final DoubleSupplier vxMetersPerSec;

    // Tune these
    private static final double kP_strafe_mps_per_deg = 0.03; // m/s per degree of tx
    private static final double kMaxStrafe_mps = 0.6;
    private static final double kP_omega_radps_per_deg = 0.06; // rad/s per degree of tx
    private static final double kMaxOmega_radps = 2.0;
    private static final double kDeadband_deg = 1.0;

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public ReefStrafeAlign(CommandSwerveDrivetrain drivetrain,
            Limelight4 limelight,
            DoubleSupplier vxMetersPerSec) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.vxMetersPerSec = vxMetersPerSec;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double txDeg = limelight.getTxDegrees();

        // Deadband so we don't jitter
        if (Math.abs(txDeg) < kDeadband_deg) {
            drivetrain.setControl(request.withVelocityX(vxMetersPerSec.getAsDouble())
                    .withVelocityY(0)
                    .withRotationalRate(0));
            return;
        }

        // NOTE: Signs may need flipping depending on your conventions.
        // You already confirmed rotate sign with RotateToTag (you used -kP * txDeg).
        double omegaRadps = MathUtil.clamp(-kP_omega_radps_per_deg * txDeg, -kMaxOmega_radps, kMaxOmega_radps);

        // For strafe: if tx is positive (tag to the right), strafe right (positive Y)
        // to center it.
        // If your robot strafes the wrong way, flip the sign on this line.
        double vyMps = MathUtil.clamp(+kP_strafe_mps_per_deg * txDeg, -kMaxStrafe_mps, kMaxStrafe_mps);

        drivetrain.setControl(
                request.withVelocityX(vxMetersPerSec.getAsDouble())
                        .withVelocityY(vyMps)
                        .withRotationalRate(omegaRadps));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
