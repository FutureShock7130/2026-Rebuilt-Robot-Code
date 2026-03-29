// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.kMaxSpeed;
import static frc.robot.Constants.kMaxAngularRate;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignTower;
import frc.robot.commands.ManualShooterIndexer;
import frc.robot.commands.ShooterIndexer;
import frc.robot.commands.SwerveWithAim;
import frc.robot.commands.TestShooterIndexer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(kMaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Climber climber = new Climber();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private final SwerveWithAim drive = new SwerveWithAim(
        drivetrain,
        () -> MathUtil.copyDirectionPow(-joystick.getLeftY(), 2.0) * kMaxSpeed,
        () -> MathUtil.copyDirectionPow(-joystick.getLeftX(), 2.0) * kMaxSpeed,
        () -> -joystick.getRightX() * kMaxAngularRate * SmartDashboard.getNumber("AngularRate", 1.0),
        joystick.rightBumper()
    );

    private final AlignTower autoClimb = new AlignTower(
        drivetrain,
        () -> MathUtil.copyDirectionPow(-joystick.getLeftY(), 2.0) * kMaxSpeed / 3.0,
        () -> MathUtil.copyDirectionPow(-joystick.getLeftX(), 2.0) * kMaxSpeed / 3.0,
        () -> -joystick.getRightX()
    );

    private final ShooterIndexer shooterIndexer = new ShooterIndexer(
        shooter,
        indexer,
        joystick.rightBumper(),
        joystick.rightTrigger(),
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds
    );

    private final ManualShooterIndexer manualShooterIndexer = new ManualShooterIndexer(
        shooter,
        indexer,
        joystick.getHID(),
        operatorJoystick.getHID()
    );

    private final TestShooterIndexer testShooterIndexer = new TestShooterIndexer(
        shooter,
        indexer,
        joystick.getHID(),
        () -> drivetrain.getState().Pose
    );

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Intake", intake.intake());
        NamedCommands.registerCommand(
            "Shoot", 
            Commands.parallel(
                Commands.parallel(new SwerveWithAim(drivetrain), new ShooterIndexer(shooter, indexer, () -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds)),
                Commands.waitSeconds(1.0).andThen(intake.toDefaultState())
            ).withTimeout(3.0)
        );

        new EventTrigger("Accel").whileTrue(Commands.run(() -> shooter.setSpeed(24, 46)).withName("Accel"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        SmartDashboard.putData("Subsystems/Climber", climber);
        SmartDashboard.putData("Subsystems/Indexer", indexer);
        SmartDashboard.putData("Subsystems/Intake", intake);
        SmartDashboard.putData("Subsystems/Shooter", shooter);

        SmartDashboard.putNumber("AngularRate", 1.5);
    }

    private void configureBindings() {
        joystick.start().and(joystick.back()).onTrue(Commands.runOnce(() -> SignalLogger.stop()));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drive);
        shooter.setDefaultCommand(shooterIndexer);
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("SwerveIdle")
        );

        // RobotModeTriggers.teleop().onTrue(climber.toPreClimbPos());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.leftTrigger().toggleOnTrue(intake.intake());
        joystick.povUp().whileTrue(intake.outTake());
        joystick.povDown().onTrue(intake.toDefaultState());
        joystick.povRight().onTrue(intake.toFeedingState());
        joystick.rightTrigger().debounce(1.0).onTrue(intake.toDefaultState());

        joystick.x().onTrue(climber.toPreClimbPos());
        joystick.y().onTrue(climber.toDefaultPose());
        joystick.a().whileTrue(autoClimb);

        // debug/test/manual mode trigger
        joystick.povLeft().toggleOnTrue(testShooterIndexer);
        operatorJoystick.start().onTrue(manualShooterIndexer);
        operatorJoystick.back().onTrue(shooterIndexer);

        drivetrain.registerTelemetry(logger::telemeterize);
        intake.registerTelemetry(logger::telemeterizeIntake);
        shooter.registerTelemetry(logger::telemeterizeShooterAngle, logger::telemeterizeShooterSpeeds);
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
        ).withTimeout(timeSeconds).withName("Rumble");
    }
}
