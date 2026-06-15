package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionAlign_Debug extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Start with a base request
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    public VisionAlign_Debug(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain); // REQUIRED for interrupting TeleopDrive
    }

    @Override
    public void initialize() {
        System.out.println("[VisionAlign_Debug] Initialized");
    }

    @Override
    public void execute() {

        System.out.println("[VisionAlign_Debug] EXECUTING");

        Rotation2d yawRot = vision.getTargetYaw();
        double yaw = yawRot.getDegrees();
        boolean hasTarget = vision.hasTarget();

        System.out.println("[VisionAlign_Debug] hasTarget=" + hasTarget + " yaw=" + yaw);

        if (hasTarget) {
            // Reassign the request (requests are immutable)
            request = request
                    .withVelocityX(0)
                    .withVelocityY(0.3)
                    .withRotationalRate(0);

            System.out.println("[VisionAlign_Debug] DRIVING: Y=0.3");
        } else {
            request = request
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);

            System.out.println("[VisionAlign_Debug] no target → stop");
        }

        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[VisionAlign_Debug] End");
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}
