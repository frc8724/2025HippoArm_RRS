// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
// === Added imports for Hippo Arm ===
import frc.robot.subsystems.Arm;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.WaveArmCommand;
import frc.robot.commands.VisionAlign;

import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // === Two controllers ===
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // === Subsystems ===
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Arm arm = new Arm();

    /* Setting up Vision Subsystem */
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final VisionAlign visionAlignCommand = new VisionAlign(drivetrain, visionSubsystem);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Vision
        driverController.leftTrigger(0.2).whileTrue(visionAlignCommand);

        // === Drivetrain controls (driver controller) ===
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        // Idle mode when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Basic swerve controls
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
                        -driverController.getLeftX()))));

        driverController.pov(0)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverController.pov(180)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // SysId bindings
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // === Arm control bindings (operator controller) ===

        // Preset buttons
        operatorController.a().onTrue(new MoveArmToPosition(arm, 10.0));
        operatorController.b().onTrue(new MoveArmToPosition(arm, 90.0));
        operatorController.y().onTrue(new MoveArmToPosition(arm, 110.0));
        operatorController.rightTrigger().whileTrue(new WaveArmCommand(arm));

        // Return to zero position (horizontal) on Back button
        operatorController.back().onTrue(new MoveArmToPosition(arm, 0.0));

        // === Manual control with right stick Y-axis (open-loop) ===
        arm.setDefaultCommand(
                new RunCommand(() -> {
                    double stick = -operatorController.getRightY(); // invert so forward = arm up
                    arm.setPercent(stick); // direct open-loop control
                }, arm));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
