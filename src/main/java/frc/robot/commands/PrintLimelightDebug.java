package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight4;

public class PrintLimelightDebug extends Command {
    private final Limelight4 limelight;

    public PrintLimelightDebug(Limelight4 limelight) {
        this.limelight = limelight;
        addRequirements(limelight); // tells the scheduler this command uses the Limelight subsystem
    }

    @Override
    public void initialize() {
        // Nothing special to do when the command starts
    }

    @Override
    public void execute() {
        // Push some basic Limelight info to SmartDashboard every loop
        SmartDashboard.putBoolean("LL4/hasTarget", limelight.hasTarget());
        SmartDashboard.putNumber("LL4/tx", limelight.getTxDegrees());
        SmartDashboard.putNumber("LL4/ty", limelight.getTyDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up when the command ends
    }

    @Override
    public boolean isFinished() {
        // Keep running until interrupted (e.g., while a button is held)
        return false;
    }
}
