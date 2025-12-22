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
    private static final double kP_radPerSecPerDeg = 0.06; // rad/s per degree
    private static final double kMaxOmega_radPerSec = 2.0; // cap rotation speed
    private static final double kDeadband_deg = 1.0; // stop within ±1 deg

    // If robot is physically aligned but tx reads -0.7, set this to -0.7
    private static final double kTxOffsetDeg = -0.7;

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
    public void initialize() {
        // nothing
    }

    @Override
    public void execute() {
        // Safe: if no tag, don't rotate
        if (!limelight.hasTarget()) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Degrees from Limelight wrapper
        double txDeg = limelight.getTxDegrees();

        // Apply offset so "physically aligned" corresponds to error=0
        double errorDeg = txDeg - kTxOffsetDeg;

        // Deadband prevents hunting
        if (Math.abs(errorDeg) <= kDeadband_deg) {
            drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // P-control (if it turns the wrong way, flip the sign here)
        double omegaRadPerSec = kP_radPerSecPerDeg * errorDeg;
        // double omegaRadPerSec = -kP_radPerSecPerDeg * errorDeg;

        omegaRadPerSec = MathUtil.clamp(omegaRadPerSec, -kMaxOmega_radPerSec, kMaxOmega_radPerSec);

        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRadPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false; // while held
    }
}
