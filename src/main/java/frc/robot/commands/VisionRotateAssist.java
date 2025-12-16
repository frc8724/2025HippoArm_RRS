package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight4;

import java.util.function.DoubleSupplier;

public class VisionRotateAssist extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight4 limelight;

    // Driver translation inputs (m/s) supplied from RobotContainer (joysticks ->
    // scaled)
    private final DoubleSupplier vxMetersPerSec;
    private final DoubleSupplier vyMetersPerSec;

    // --- Tune on carpet ---
    private static final double kP_radPerSecPerDeg = 0.06;
    private static final double kMaxOmega_radPerSec = 2.0;
    private static final double kDeadband_deg = 1.0;

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public VisionRotateAssist(
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
    public void execute() {
        double omegaRadPerSec = 0.0;

        if (limelight.hasTarget()) {
            double txDeg = limelight.getTxDegrees();

            if (Math.abs(txDeg) > kDeadband_deg) {
                // NOTE: using the SAME sign convention as your working RotateToTag and
                // ApproachTagStraight
                omegaRadPerSec = MathUtil.clamp(
                        -kP_radPerSecPerDeg * txDeg,
                        -kMaxOmega_radPerSec,
                        kMaxOmega_radPerSec);
            }
        }

        drivetrain.setControl(
                request.withVelocityX(vxMetersPerSec.getAsDouble())
                        .withVelocityY(vyMetersPerSec.getAsDouble())
                        .withRotationalRate(omegaRadPerSec));
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }
}
