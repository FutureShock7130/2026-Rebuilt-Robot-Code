package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;
import static frc.robot.Constants.FieldConstants.kHubLocation;
import static frc.robot.Constants.FieldConstants.kLeftTransportTarget;
import static frc.robot.Constants.FieldConstants.kRightTransportTarget;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            .withHeadingPID(12, 0, 0.3)
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
            doAimSupplier.getAsBoolean()
        ) {
            // Switch aiming target between HUB and transport target based on robot's position
            Translation2d facingTarget;
            if ((robotPose.getX() - AllianceFlipUtil.flipX(0)) * (robotPose.getX() - AllianceFlipUtil.flipX(4.028)) < 0) { // in alliance zones
                facingTarget = kHubLocation;
            } else {
                facingTarget = AllianceFlipUtil.flipY(robotPose.getY()) < 4.035 ? kRightTransportTarget : kLeftTransportTarget;
            }

            Rotation2d angle = AllianceFlipUtil.flip(facingTarget).minus(robotPose.getTranslation()).getAngle();
            // Set modules to "X" fasion when near facing angle target and no driver inputs
            if (
                Math.abs(angle.getDegrees() - robotPose.getRotation().getDegrees()) > 1.5
                    || Math.abs(xSpeedSupplier.getAsDouble()) > 0
                    || Math.abs(ySpeedSupplier.getAsDouble()) > 0
            ) {
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
}