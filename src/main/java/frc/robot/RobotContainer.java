package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Arm;

// Commands
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.WaveArmCommand;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.VisionAlignShoot;
import frc.robot.commands.VisionAlign_Debug;
import frc.robot.commands.ReefTargetSide;

// Telemetry
import frc.robot.Telemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2;

    /* Swerve request templates */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Arm arm = new Arm();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    // Auto chooser
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warm PathPlanner to avoid initial stutter
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // =============================================================
        // DRIVER VISION COMMANDS
        // =============================================================

        // Left Trigger → Align to LEFT reef post
        driverController.leftTrigger(0.2).whileTrue(
                new VisionAlign(drivetrain, visionSubsystem, ReefTargetSide.LEFT));

        // Right Trigger → Align to RIGHT reef post
        driverController.rightTrigger(0.2).whileTrue(
                new VisionAlign(drivetrain, visionSubsystem, ReefTargetSide.RIGHT));

        // Y Button → Aim shooter at CENTER of reef (rotation + strafe)
        driverController.y().whileTrue(
                new VisionAlignShoot(drivetrain, visionSubsystem));

        // =============================================================
        // DEFAULT DRIVING CONTROL
        // =============================================================

        drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> drivetrain.setControl(
                                drive
                                        .withVelocityX(driverController.getLeftY() * (MaxSpeed/2))
                                        .withVelocityY(driverController.getLeftX() * (MaxSpeed/2))
                                        .withRotationalRate(-driverController.getRightX() * (MaxAngularRate/2))
                        ),
                        drivetrain));

        // Idle mode when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Brake mode
        driverController.a().whileTrue(
                drivetrain.applyRequest(() -> brake));

        // Point wheels at stick direction
        driverController.b().whileTrue(
                drivetrain.applyRequest(() -> point
                        .withModuleDirection(new Rotation2d(
                                -driverController.getLeftY(),
                                -driverController.getLeftX()))));

        // Test drive
        driverController.x().onTrue(new TestDriveCommand(drivetrain));

        // VisionAlign debugging (unchanged)
        driverController.y().whileTrue(
                new VisionAlign_Debug(drivetrain, visionSubsystem));

        // POV straight-line assists
        driverController.pov(0).whileTrue(
                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverController.pov(180).whileTrue(
                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // SysId tests
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset heading
        driverController.leftBumper().onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Push telemetry to NT
        drivetrain.registerTelemetry(logger::telemeterize);

        // =============================================================
        // OPERATOR ARM CONTROLS (unchanged)
        // =============================================================

        operatorController.a().onTrue(new MoveArmToPosition(arm, 10.0));
        operatorController.b().onTrue(new MoveArmToPosition(arm, 90.0));
        operatorController.y().onTrue(new MoveArmToPosition(arm, 110.0));
        operatorController.rightTrigger().whileTrue(new WaveArmCommand(arm));
        operatorController.back().onTrue(new MoveArmToPosition(arm, 0.0));

        // Manual joystick control
        arm.setDefaultCommand(
                new RunCommand(() -> {
                    double stickY = -operatorController.getRightY();
                    arm.setPercent(stickY);
                }, arm));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
