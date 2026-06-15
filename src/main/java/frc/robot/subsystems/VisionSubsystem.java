package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable ll = NetworkTableInstance.getDefault().getTable("limelight");

    // Simple accessors
    public boolean hasTarget() {
        return ll.getEntry("tv").getDouble(0) == 1.0;
    }

    public double getTx() {
        return ll.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return ll.getEntry("ty").getDouble(0.0);
    }

    public double getTa() {
        return ll.getEntry("ta").getDouble(0.0);
    }

    public int getTagId() {
        return (int) ll.getEntry("tid").getDouble(-1);
    }

    // Optional smoothing (useful for noisy LL2+)
    public double getFilteredTx(double lastTx, double alpha) {
        double newTx = getTx();
        return (alpha * newTx) + ((1 - alpha) * lastTx);
    }
}
