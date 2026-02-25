package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;
import static frc.robot.Constants.FieldConstants.kHubLocation;
import static frc.robot.Constants.FieldConstants.kTransportTarget_Left;
import static frc.robot.Constants.FieldConstants.kTransportTarget_Right;
import static frc.robot.Constants.FieldConstants.kLeftTrenchStartPoint_Blue;
import static frc.robot.Constants.FieldConstants.kRightTrenchStartPoint_Blue;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterCompensator;

public class SwerveWithAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final ShooterCompensator shooterCompensator = new ShooterCompensator();

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier, doCenteringSupplier, doTransportSupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withMaxAbsRotationalRate(kMaxAngularRate)
            .withHeadingPID(12, 0, 0.3)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private Translation2d autoCenteringVelocity = new Translation2d(0, 0);

    private final PIDController yPID = new PIDController(8.5, 0, 0.3);

    private final Translation2d kTrench = new Translation2d(2.7, 0);
    private final Translation2d u = kTrench.div(kTrench.getNorm());
    private final Translation2d n = new Translation2d(-u.getY(), u.getX());

    /** Command for aiming during auto */
    public SwerveWithAim(CommandSwerveDrivetrain drivetrain) {
        this(
                drivetrain,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                () -> true,
                () -> false,
                () -> false);
    }

    public SwerveWithAim(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSpeedSupplier,
            BooleanSupplier doAimSupplier,
            BooleanSupplier doCenteringSupplier,
            BooleanSupplier doTransportSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        this.doCenteringSupplier = doCenteringSupplier;
        this.doTransportSupplier = doTransportSupplier;
        addRequirements(drivetrain);

        yPID.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds;

        autoCenteringVelocity = n
                .times(
                        yPID.calculate(n.dot(robotPose.getTranslation().minus(getCloserTrenchStartPoint()))))
                .plus(u.times(xSpeedSupplier.getAsDouble()));

        if (doAimSupplier.getAsBoolean() &&
                (robotPose.getX() - AllianceFlipUtil.flipX(0)) * (robotPose.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in
                                                                                                                        // alliance
                                                                                                                        // zones
        ) {
            // using ShooterCompensator
            Rotation2d compensatedAngle = shooterCompensator.calculateCompensatedHeading(
                    robotPose,
                    fieldSpeeds,
                    AllianceFlipUtil.flip(kHubLocation));

            // using simple angle to target
            Rotation2d angle = AllianceFlipUtil.flip(kHubLocation).minus(robotPose.getTranslation()).getAngle();
            if (Math.abs(angle.getDegrees() - robotPose.getRotation().getDegrees()) > 1.5
                    || Math.abs(xSpeedSupplier.getAsDouble()) > 0 || Math.abs(ySpeedSupplier.getAsDouble()) > 0) {
                drivetrain.setControl(
                        driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                                .withVelocityY(ySpeedSupplier.getAsDouble())
                                .withTargetDirection(angle));
            } else {
                drivetrain.setControl(brake);
            }
        } else if (doCenteringSupplier.getAsBoolean()) {
            drivetrain.setControl(
                    driveAngle.withVelocityX(autoCenteringVelocity.getX())
                            .withVelocityY(autoCenteringVelocity.getY())
                            .withTargetDirection(AllianceFlipUtil.flip(Rotation2d.fromDegrees(0))));

        } else if (doTransportSupplier.getAsBoolean()) {
            Rotation2d angle_Transport = AllianceFlipUtil
                    .flip(getCloserTrenchStartPoint() == AllianceFlipUtil.flip(kLeftTrenchStartPoint_Blue)
                            ? kTransportTarget_Left
                            : kTransportTarget_Right)
                    .minus(robotPose.getTranslation()).getAngle();
            drivetrain.setControl(
                    driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                            .withVelocityY(ySpeedSupplier.getAsDouble())
                            .withTargetDirection(angle_Transport));
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

    private boolean isInTrenchZone(Pose2d robotPose) {
        Translation2d rightStartFlipped = AllianceFlipUtil.flip(kRightTrenchStartPoint_Blue);
        Translation2d leftStartFlipped = AllianceFlipUtil.flip(kLeftTrenchStartPoint_Blue);

        Translation2d rightEnd_Blue = new Translation2d(
                kRightTrenchStartPoint_Blue.getX() + kTrench.getX(), 0);
        double flippedEndX = AllianceFlipUtil.flip(rightEnd_Blue).getX();

        double trenchMinX = Math.min(rightStartFlipped.getX(), flippedEndX);
        double trenchMaxX = Math.max(rightStartFlipped.getX(), flippedEndX);

        double rightTrenchY = rightStartFlipped.getY();
        double leftTrenchY = leftStartFlipped.getY();
        double yTolerance = 1.5;

        boolean inRightTrench = (robotPose.getX() > trenchMinX && robotPose.getX() < trenchMaxX) &&
                Math.abs(robotPose.getY() - rightTrenchY) < yTolerance;

        boolean inLeftTrench = (robotPose.getX() > trenchMinX && robotPose.getX() < trenchMaxX) &&
                Math.abs(robotPose.getY() - leftTrenchY) < yTolerance;

        return inRightTrench || inLeftTrench;
    }

    private Translation2d getCloserTrenchStartPoint() {
        Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();

        Translation2d actualLeftStart = AllianceFlipUtil.flip(kLeftTrenchStartPoint_Blue);
        Translation2d actualRightStart = AllianceFlipUtil.flip(kRightTrenchStartPoint_Blue);

        double robotToLeftTrench = robotTranslation.getDistance(actualLeftStart);
        double robotToRightTrench = robotTranslation.getDistance(actualRightStart);

        return robotToLeftTrench < robotToRightTrench ? actualLeftStart : actualRightStart;
    }
}