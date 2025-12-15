package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

public class RotateToTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;

    // Tune on carpet
    private static final double kP_radPerSecPerDeg = 0.06; // rad/s per degree of tx
    private static final double kMaxOmega_radPerSec = 2.0; // cap rotation speed
    private static final double kDeadband_deg = 1.0; // stop within ±1 deg

    // Reusable request object (Robot-centric, rotate in place)
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public RotateToTag(CommandSwerveDrivetrain drivetrain, Limelight4 limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Safe: if no tag, don't rotate
        if (!limelight.hasTarget()) {
            drivetrain.setControl(
                    request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double txDeg = limelight.getTxDegrees();

        // Deadband prevents hunting
        if (Math.abs(txDeg) <= kDeadband_deg) {
            drivetrain.setControl(
                    request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // P-control on tx: drive tx -> 0
        double omegaRadPerSec = MathUtil.clamp(-kP_radPerSecPerDeg * txDeg, -kMaxOmega_radPerSec, kMaxOmega_radPerSec);

        drivetrain.setControl(
                request.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRadPerSec));
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }
}
