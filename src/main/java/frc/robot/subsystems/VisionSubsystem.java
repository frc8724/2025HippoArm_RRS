package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static final String LL = "limelight-front";

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTarget", LimelightHelpers.getTV(LL));
        SmartDashboard.putNumber("Vision/tx", LimelightHelpers.getTX(LL));
        SmartDashboard.putNumber("Vision/ty", LimelightHelpers.getTY(LL));
        SmartDashboard.putNumber("Vision/ta", LimelightHelpers.getTA(LL));
    }
}
