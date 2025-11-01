package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * Command to wave the arm between 80° and 100° repeatedly while held.
 * When released, returns the arm to 0°.
 */
public class WaveArmCommand extends Command {
    private final Arm arm;
    private final Timer timer = new Timer();
    private boolean goingUp = true;

    public WaveArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double cycleTime = 1.0; // seconds for each half wave
        if (timer.hasElapsed(cycleTime)) {
            goingUp = !goingUp;
            timer.restart();
        }

        double target = goingUp ? 100.0 : 80.0;
        arm.setTargetDegrees(target);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setTargetDegrees(0.0); // Return to zero when trigger released
    }

    @Override
    public boolean isFinished() {
        return false; // runs continuously until trigger released
    }
}
