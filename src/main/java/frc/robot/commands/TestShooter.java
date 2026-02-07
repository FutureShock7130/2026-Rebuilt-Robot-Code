package frc.robot.commands;

import static frc.robot.Constants.kMaxSpeed;
import static frc.robot.Constants.kMaxAngularRate;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class TestShooter extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Indexer indexer;

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier, doShootSupplier, doPrintValuesSupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.02).withRotationalDeadband(kMaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(kMaxSpeed * 0.02) // Add a 10% deadband
            .withMaxAbsRotationalRate(kMaxAngularRate)
            .withHeadingPID(12, 0, 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Translation2d target = new Translation2d(4.625, 4.034);

    private String testResults = "";

    public TestShooter(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotSpeedSupplier,
        BooleanSupplier doAimSupplier,
        BooleanSupplier doShootSupplier,
        BooleanSupplier doPrintValuesSupplier
    ) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.doPrintValuesSupplier = doPrintValuesSupplier;
        addRequirements(drivetrain, shooter, indexer);
    }

    @Override
    public void initialize() {
        // don't remove this reduant stuff
        doPrintValuesSupplier.getAsBoolean();
        SmartDashboard.putNumber("TestShooter/Angle", 0.0);
        SmartDashboard.putNumber("TestShooter/UpSpeed", 0.0);
        SmartDashboard.putNumber("TestShooter/DownSpeed", 0.0);
        testResults = "";
    }

    @Override
    public void execute() {
        Pose2d curr = drivetrain.getState().Pose;
        Translation2d diff = AllianceFlipUtil.flip(target).minus(curr.getTranslation());
        Rotation2d angle = diff.getAngle();
        double distance = diff.getNorm();

        if (
            (curr.getX() - AllianceFlipUtil.flipX(0)) * (curr.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in alliance zon
            && doAimSupplier.getAsBoolean()
        ) {
            if (Math.abs(angle.getDegrees() - curr.getRotation().getDegrees()) > 3 || Math.abs(xSpeedSupplier.getAsDouble()) > 0.1 || Math.abs(ySpeedSupplier.getAsDouble()) > 0.1) {
                drivetrain.setControl(
                    driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                        .withVelocityY(ySpeedSupplier.getAsDouble())
                        .withTargetDirection(angle)
                );
            } else {
                drivetrain.setControl(brake);
            }

            shooter.setAngle(SmartDashboard.getNumber("TestShooter/Angle", 0.0));
            shooter.set(SmartDashboard.getNumber("TestShooter/UpSpeed", 0.0), SmartDashboard.getNumber("TestShooter/DownSpeed", 0.0));

            if (doShootSupplier.getAsBoolean() && shooter.atTarget()) {
                indexer.set(1, 1);
            } else {
                indexer.set(0, 0);
            }
        } else {
            drivetrain.setControl(
                drive.withVelocityX(xSpeedSupplier.getAsDouble())
                    .withVelocityY(ySpeedSupplier.getAsDouble())
                    .withRotationalRate(rotSpeedSupplier.getAsDouble())
            );

            shooter.setAngle(0);
            shooter.set(0, 0);
            indexer.set(0, 0);
        }

        if (doPrintValuesSupplier.getAsBoolean()) {
            String formatted = String.format("%.2f", distance);
            testResults += "distance=" + formatted + ",angle=" + SmartDashboard.getNumber("TestShooter/Angle", 0.0) + ",upSpeed=" + SmartDashboard.getNumber("TestShooter/UpSpeed", 0.0) + ",downSpeed=" + SmartDashboard.getNumber("TestShooter/DownSpeed", 0.0) + "\n";
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        shooter.set(0, 0);
        shooter.setAngle(0);
        indexer.stopAll();
        System.out.println(testResults);
        SmartDashboard.putString("TestShooter/Result", testResults);
    }
}
