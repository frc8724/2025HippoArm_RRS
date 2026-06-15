package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.VisionHelpers;

public class VisionAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final double offsetDeg;

    // Local request object so we don't reallocate every cycle
    private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, double offsetDeg) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.offsetDeg = offsetDeg;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        if (!vision.hasTarget()) {
            drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }

        double tx = vision.getTx();
        double error = tx - offsetDeg;

        double turn = error * VisionHelpers.kP_TURN;
        double forward = VisionHelpers.FORWARD_SPEED;

        drivetrain.setControl(
            request.withSpeeds(
                new ChassisSpeeds(
                    forward,   // robot forward
                    0.0,       // no strafe in Version 1
                    turn       // rotate to align
                )
            )
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getTx() - offsetDeg) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
