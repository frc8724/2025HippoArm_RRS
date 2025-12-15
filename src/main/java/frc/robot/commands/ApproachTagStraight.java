package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

public class ApproachTagStraight extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;

    // --- Tune on carpet ---
    private static final double kForwardMps = 0.35; // slow creep forward
    private static final double kP_radPerSecPerDeg = 0.06; // SAME family as RotateToTag
    private static final double kMaxOmega_radPerSec = 2.0;
    private static final double kDeadband_deg = 1.0;

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public ApproachTagStraight(CommandSwerveDrivetrain drivetrain, Limelight4 limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Safety: if no target, stop
        if (!limelight.hasTarget()) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double txDeg = limelight.getTxDegrees();

        // Rotate to drive tx -> 0 (same sign convention you proved works in
        // RotateToTag!)
        double omegaRadPerSec = 0.0;
        if (Math.abs(txDeg) > kDeadband_deg) {
            omegaRadPerSec = MathUtil.clamp(
                    -kP_radPerSecPerDeg * txDeg, // NOTE the minus sign (match your working RotateToTag)
                    -kMaxOmega_radPerSec,
                    kMaxOmega_radPerSec);
        }

        drivetrain.setControl(
                request.withVelocityX(-kForwardMps)
                        .withVelocityY(0)
                        .withRotationalRate(omegaRadPerSec));
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }
}
