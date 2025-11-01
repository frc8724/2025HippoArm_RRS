package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * Command to move the arm to a specific target angle (in degrees).
 * Uses the Arm subsystem to command a position and ends when within tolerance.
 */
public class MoveArmToPosition extends Command {
    private final Arm arm;
    private final double targetAngle;

    public MoveArmToPosition(Arm arm, double targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetDegrees(targetAngle);
    }

    @Override
    public boolean isFinished() {
        double error = Math.abs(arm.getArmAngle() - targetAngle);
        return error < 2.0; // finish when within 2 degrees of target
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.stop();
        }
    }
}
