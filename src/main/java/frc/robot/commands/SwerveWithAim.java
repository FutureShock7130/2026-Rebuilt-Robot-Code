package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;
import static frc.robot.Constants.FieldConstants.kHubLocation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveWithAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withMaxAbsRotationalRate(kMaxAngularRate)
            .withHeadingPID(12, 0, 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /** Command for aiming during auto */
    public SwerveWithAim(CommandSwerveDrivetrain drivetrain) {
        this(
            drivetrain,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> true
        );
    }

    public SwerveWithAim(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotSpeedSupplier,
        BooleanSupplier doAimSupplier
    ) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        if (
            doAimSupplier.getAsBoolean() &&
            (robotPose.getX() - AllianceFlipUtil.flipX(0)) * (robotPose.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in alliance zones
        ) {
            Rotation2d angle = AllianceFlipUtil.flip(kHubLocation).minus(robotPose.getTranslation()).getAngle();
            if (Math.abs(angle.getDegrees() - robotPose.getRotation().getDegrees()) > 3 || Math.abs(xSpeedSupplier.getAsDouble()) > 0 || Math.abs(ySpeedSupplier.getAsDouble()) > 0) {
                drivetrain.setControl(
                    driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                        .withVelocityY(ySpeedSupplier.getAsDouble())
                        .withTargetDirection(angle)
                );
            } else {
                drivetrain.setControl(brake);
            }
        } else {
            drivetrain.setControl(
                drive.withVelocityX(xSpeedSupplier.getAsDouble())
                    .withVelocityY(ySpeedSupplier.getAsDouble())
                    .withRotationalRate(rotSpeedSupplier.getAsDouble())
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(null);
    }
}