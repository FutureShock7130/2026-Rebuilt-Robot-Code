// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.kMaxSpeed;
import static frc.robot.Constants.kMaxAngularRate;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AllInOne;
import frc.robot.commands.TestShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final SwerveRequest.FieldCentric driveSlow = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.02).withRotationalDeadband(kMaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(kMaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Climber climber = new Climber();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private final AllInOne drive = new AllInOne(
        drivetrain,
        shooter,
        indexer,
        () -> -joystick.getLeftY() * kMaxSpeed, // Drive forward with negative Y (forward)
        () -> -joystick.getLeftX() * kMaxSpeed, // Drive left with negative X (left)
        () -> -joystick.getRightX() * kMaxAngularRate, // Drive counterclockwise with negative X (left)
        () -> joystick.getHID().getRightBumperButton(),
        () -> joystick.getHID().getRightTriggerAxis() > 0.5
    );

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Intake", intake.set(IntakeConstants.kIntakeSpeed));
        NamedCommands.registerCommand("Shoot", new AllInOne(drivetrain, shooter, indexer, () -> 0, () -> 0, () -> 0, () -> true, () -> true).withTimeout(3.5));
        NamedCommands.registerCommand("PreClimb", climber.preClimb().until(climber::atTargetHeight));
        NamedCommands.registerCommand("Climb", climber.climb());

        new EventTrigger("Accel").whileTrue(Commands.runOnce(() -> shooter.set(5, 5)));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drive);
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // RobotModeTriggers.teleop().onTrue(climber.preClimb().andThen(Commands.waitSeconds(5)).andThen(climber.climb()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.back().and(joystick.b()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.a()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.b()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.a()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        joystick.a().and(joystick.b()).onTrue(Commands.runOnce(() -> SignalLogger.stop()));

        joystick.start().and(joystick.back()).toggleOnTrue(
            new TestShooter(
                drivetrain,
                shooter,
                indexer,
                () -> -joystick.getLeftY() * kMaxSpeed,
                () -> -joystick.getLeftX() * kMaxSpeed,
                () -> -joystick.getRightX() * kMaxAngularRate,
                () -> joystick.getHID().getRightBumperButton(),
                () -> joystick.getHID().getRightTriggerAxis() > 0.5,
                () -> joystick.getHID().getAButtonPressed()
            )
        );

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.leftTrigger().whileTrue(intake.set(IntakeConstants.kIntakeSpeed));

        joystick.povUp()
            .toggleOnTrue(climber.extendThenClimb())
            .toggleOnTrue(drivetrain.applyRequest(
                () -> 
                    driveSlow.withVelocityX(-joystick.getLeftY() * 1.5)
                        .withVelocityY(-joystick.getLeftX() * 1.5)
                        .withRotationalRate(-joystick.getRightX() * 1)
            ));

        drivetrain.registerTelemetry(logger::telemeterize);

        shooter.registerTelemetry(logger::telemeterizeShooter);

        climber.registerTelemetry(logger::telemeterizeClimber);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public Command rumble(double strength, double timeSeconds) {
        return Commands.runEnd(
            () -> joystick.setRumble(RumbleType.kBothRumble, strength),
            () -> joystick.setRumble(RumbleType.kBothRumble, 0)
        ).withTimeout(timeSeconds);
    }
}
