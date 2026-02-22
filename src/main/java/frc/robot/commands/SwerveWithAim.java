package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;
import static frc.robot.Constants.FieldConstants.kHubLocation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveWithAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier, doCenteringSupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withMaxAbsRotationalRate(kMaxAngularRate)
            .withHeadingPID(12, 0, 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private Translation2d autoCenteringVelocity = new Translation2d(0, 0);

    private final PIDController yPID = new PIDController(8.5, 0, 0.8);

    private final Translation2d kRightTrenchStartPoint = new Translation2d(3.4, 0.7);
    private final Translation2d kTrench = new Translation2d(2.3, 0);
    private final Translation2d u = kTrench.div(kTrench.getNorm());
    private final Translation2d n = new Translation2d(-u.getY(), u.getX());

    private final Translation2d kRightTrenchMidPoint = new Translation2d(4.6, 0);

    /** Command for aiming during auto */
    public SwerveWithAim(CommandSwerveDrivetrain drivetrain) {
        this(
                drivetrain,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                () -> true,
                () -> false);
    }

    public SwerveWithAim(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSpeedSupplier,
            BooleanSupplier doAimSupplier,
            BooleanSupplier doCenteringSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        this.doCenteringSupplier = doCenteringSupplier;
        addRequirements(drivetrain);

        yPID.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;

        autoCenteringVelocity = n
                .times(
                        yPID.calculate(n.dot(robotPose.getTranslation().minus(kRightTrenchStartPoint))))
                .plus(u.times(xSpeedSupplier.getAsDouble()));

        if (doAimSupplier.getAsBoolean() &&
                (robotPose.getX() - AllianceFlipUtil.flipX(0)) * (robotPose.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in
                                                                                                                        // alliance
                                                                                                                        // zones
        ) {
            Rotation2d angle = AllianceFlipUtil.flip(kHubLocation).minus(robotPose.getTranslation()).getAngle();
            if (Math.abs(angle.getDegrees() - robotPose.getRotation().getDegrees()) > 3
                    || Math.abs(xSpeedSupplier.getAsDouble()) > 0 || Math.abs(ySpeedSupplier.getAsDouble()) > 0) {
                drivetrain.setControl(
                        driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                                .withVelocityY(ySpeedSupplier.getAsDouble())
                                .withTargetDirection(angle));
            } else {
                drivetrain.setControl(brake);
            }
        } else if (robotPose.getTranslation().minus(kRightTrenchMidPoint).getNorm() < 3.0
                && xSpeedSupplier.getAsDouble() > 2.5
                // && doCenteringSupplier.getAsBoolean()
                ) {
                    System.out.println("auto centering");
            drivetrain.setControl(
                    driveAngle.withVelocityX(autoCenteringVelocity.getX())
                            .withVelocityY(autoCenteringVelocity.getY())
                            .withTargetDirection(AllianceFlipUtil.flip(Rotation2d.fromDegrees(0))));
        } else {
            drivetrain.setControl(
                    drive.withVelocityX(xSpeedSupplier.getAsDouble())
                            .withVelocityY(ySpeedSupplier.getAsDouble())
                            .withRotationalRate(rotSpeedSupplier.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(null);
    }
}