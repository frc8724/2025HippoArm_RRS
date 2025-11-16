package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class TestDriveCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    // Field-centric open-loop request (default mode for the free tier)
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withVelocityX(0.0) // forward/back
            .withVelocityY(0.5) // LEFT/RIGHT (strafe)
            .withRotationalRate(0.0); // rotation

    public TestDriveCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}
