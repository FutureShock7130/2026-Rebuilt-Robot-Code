package frc.robot.commands;

import static frc.robot.Constants.kMaxSpeed;
import static frc.robot.Constants.kMaxAngularRate;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AllInOne extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier, doShootSupplier, doIntakeSupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.02).withRotationalDeadband(kMaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(kMaxSpeed * 0.02)
            .withMaxAbsRotationalRate(kMaxAngularRate)
            .withHeadingPID(12, 0, 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Translation2d target = new Translation2d(4.625, 4.034);

    private final Transform3d robotToCamera = new Transform3d(new Translation3d(0.4, 0.0, 0.51), new Rotation3d(0, 0, 0));
    private final Translation2d targetGamePieceToRobot = new Translation2d(-1, 0.0);

    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap upSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap downSpeedMap = new InterpolatingDoubleTreeMap();

    public AllInOne(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotSpeedSupplier,
        BooleanSupplier doAimSupplier,
        BooleanSupplier doShootSupplier,
        BooleanSupplier doIntakeSupplier
    ) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.doIntakeSupplier = doIntakeSupplier;
        addRequirements(drivetrain, shooter, indexer, intake);

        angleMap.put(0.0, 0.03);
        angleMap.put(1.4, 0.03);
        angleMap.put(3.0, 0.04);
        angleMap.put(8.0, 0.03);

        upSpeedMap.put(0.0, 24.0);
        upSpeedMap.put(1.4, 24.0);
        upSpeedMap.put(3.0, 32.0);
        upSpeedMap.put(8.0, 24.0);

        downSpeedMap.put(0.0, 24.0);
        downSpeedMap.put(1.4, 24.0);
        downSpeedMap.put(3.0, 32.0);
        downSpeedMap.put(8.0, 24.0);
    }

    @Override
    public void execute() {
        Pose2d curr = drivetrain.getState().Pose;

        if (
            (curr.getX() - AllianceFlipUtil.flipX(0)) * (curr.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in alliance zon
            && doAimSupplier.getAsBoolean()
        ) {
            Translation2d diff = AllianceFlipUtil.flip(target).minus(curr.getTranslation());
            Rotation2d angle = diff.getAngle();
            double distance = diff.getNorm();
            if (Math.abs(angle.getDegrees() - curr.getRotation().getDegrees()) > 3 || Math.abs(xSpeedSupplier.getAsDouble()) > 0.1 || Math.abs(ySpeedSupplier.getAsDouble()) > 0.1) {
                drivetrain.setControl(
                    driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                        .withVelocityY(ySpeedSupplier.getAsDouble())
                        .withTargetDirection(angle)
                );
            } else {
                drivetrain.setControl(brake);
            }

            shooter.setAngle(angleMap.get(distance));
            shooter.set(upSpeedMap.get(distance), downSpeedMap.get(distance));

            if (doShootSupplier.getAsBoolean() && shooter.atTarget()) {
                indexer.set(1, 1);
            } else {
                indexer.set(0, 0);
            }
        } else {
            double xAdjust = 0.0, yAdjust = 0.0, zAdjust = 0.0;
            if (doIntakeSupplier.getAsBoolean()) {
                intake.set(IntakeConstants.kIntakeSpeed);
                if (LimelightHelpers.getTV("limelight-front")) {
                    Rotation2d cameraToGamePieceYaw = Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-front"));

                    // distance estimation formula : d = zDiff / tan(pitchDiff)
                    // take absloute to avoid negitive pitchDiff value
                    double distance = Math.abs(
                        (robotToCamera.getZ())
                        / Math.tan(robotToCamera.getRotation().getY() + Units.degreesToRadians(LimelightHelpers.getTY("limelight-front")))
                    );
                    // calcuate the translation and rotation diff between robot and target position.
                    Translation2d cameraToGamePiece = new Translation2d(distance, cameraToGamePieceYaw);  
                    Translation2d robotToTargetPosition = robotToCamera.getTranslation().toTranslation2d().plus(cameraToGamePiece).minus(targetGamePieceToRobot);
                    double robotToTargetRotation = robotToTargetPosition.getAngle().getRadians();
                    // Change from robot-centric speeds to field-centric speeds
                    Translation2d fieldRobotToTargetPosition = robotToTargetPosition.rotateBy(curr.getRotation());
                    xAdjust = fieldRobotToTargetPosition.getX();
                    yAdjust = fieldRobotToTargetPosition.getY();
                    zAdjust = robotToTargetRotation * 4;
                }
            }

            drivetrain.setControl(
                drive.withVelocityX(xSpeedSupplier.getAsDouble() + xAdjust)
                    .withVelocityY(ySpeedSupplier.getAsDouble() + yAdjust)
                    .withRotationalRate(rotSpeedSupplier.getAsDouble() + zAdjust)
            );

            shooter.setAngle(0);
            shooter.set(0, 0);
            indexer.set(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        shooter.set(0, 0);
        shooter.setAngle(0);
        indexer.stopAll();
    }
}
